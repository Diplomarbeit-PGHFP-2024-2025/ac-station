import asyncio
from asyncio import sleep
import datetime
import socket
import rclpy

from .minimal_publisher import MinimalPublisher

from uagents import Agent, Context
from uagents.setup import fund_agent_if_low

from aca_protocols.station_register_protocol import (
    StationRegisterResponse,
    StationRegisterRequest,
)

from aca_protocols.property_query_protocol import (
    PropertyQueryRequest,
    PropertyQueryResponse,
    PropertyData,
)

from aca_protocols.car_register_protocol import CarRegisterRequest, CarRegisterResponse

from aca_protocols.acs_registry_id import acs_id

from aca_protocols.ac_payment_protocol import MIN_TEST_AMOUNT

from aca_protocols.ac_charging_protocol import (
    CarStartedChargingInfo,
    CarFinishedChargingInfo,
)

from .payment import send_payment_request, confirm_transaction, initialize_payment_map

from aca_protocols.ac_payment_protocol import TransactionInfo

hostname = socket.gethostname()
IPAddr = socket.gethostbyname(hostname)

agent = Agent(
    name="station",
    seed="Station1",
    port=8001,
    endpoint=["http://{}:8001/submit".format(IPAddr)],
)

rclpy.init()
minimal_publisher = MinimalPublisher()

fund_agent_if_low(agent.wallet.address(), min_balance=MIN_TEST_AMOUNT)


@agent.on_event("startup")
async def startup_event(ctx: Context):
    ctx.logger.info(f"Agent: {agent.name} ({agent.address})")
    minimal_publisher.log_message(f"Agent: {agent.name} ({agent.address})")

    # run function in background so agent can fully start while registering
    asyncio.ensure_future(register_at_registry(ctx))

    properties = PropertyData(
        open_time_frames=[(0, 0), (0, 0)],
        geo_point=(20.32, 85.52),
        cost_per_kwh=34.76,
        charging_wattage=11,
        green_energy=False,
    )

    ctx.storage.set("properties", properties.to_json())
    ctx.storage.set("expireAt", datetime.datetime.fromtimestamp(86400).timestamp())

    initialize_payment_map(ctx)


async def register_at_registry(ctx: Context):
    while True:
        if (
            datetime.datetime.fromtimestamp(ctx.storage.get("expireAt"))
            > datetime.datetime.now()
        ):
            await sleep(
                ctx.storage.get("expireAt") - datetime.datetime.now().timestamp()
            )
            continue

        ctx.logger.info(f"Trying to introduce: {agent.name} ({agent.address})")

        properties = ctx.storage.get("properties")
        properties = PropertyData.from_json(properties)
        await ctx.send(
            acs_id,
            StationRegisterRequest(
                lat=properties.geo_point[0], long=properties.geo_point[1]
            ),
        )

        await sleep(5)


@agent.on_message(StationRegisterResponse)
async def on_is_registered(ctx: Context, sender: str, _msg: StationRegisterResponse):
    ctx.logger.info(f"got registered by: {sender}; TTL: {_msg.ttl}")
    ctx.storage.set("expireAt", datetime.datetime.now().timestamp() + (_msg.ttl * 0.5))


@agent.on_message(CarRegisterRequest)
async def on_register_car(ctx: Context, sender: str, msg: CarRegisterRequest):
    ctx.logger.info(f"car {sender} wants to be registered: {msg}")
    await ctx.send(sender, CarRegisterResponse(success=True))


@agent.on_message(PropertyQueryRequest)
async def on_query_properties(ctx: Context, sender: str, _msg: PropertyQueryRequest):
    ctx.logger.info(f"{sender} requested params")

    properties = ctx.storage.get("properties")

    response = PropertyQueryResponse(properties=properties)

    await ctx.send(sender, response)


@agent.on_message(CarStartedChargingInfo)
async def on_car_started_charging(
    ctx: Context, sender: str, _msg: CarStartedChargingInfo
):
    ctx.logger.info(f"car {sender} charging")


@agent.on_message(CarFinishedChargingInfo)
async def on_car_completed_charging(
    ctx: Context, sender: str, _msg: CarFinishedChargingInfo
):
    ctx.logger.info(f"car {sender} finished charging")
    await send_payment_request(ctx, sender, _msg, str(agent.wallet.address()))


@agent.on_message(TransactionInfo)
async def on_transaction_info(ctx: Context, sender: str, _msg: TransactionInfo):
    ctx.logger.info(f"car {sender} sent transaction info: {_msg}")
    await confirm_transaction(ctx, sender, _msg, str(agent.wallet.address()))


def main(args=None):
    agent.run()


if __name__ == "__main__":
    main()
