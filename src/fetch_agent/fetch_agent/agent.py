import asyncio
import datetime
import socket

import rclpy
from aca_protocols.ac_payment_protocol import MIN_TEST_AMOUNT
from aca_protocols.property_query_protocol import (
    PropertyData,
)
from uagents import Agent, Context
from uagents.setup import fund_agent_if_low

from .communication.registry import stationRegisterProtocol, register_at_registry
from .communication.car import propertyQueryProtocol, carRegisterProtocol

from .minimal_publisher import MinimalPublisher
from .payment import initialize_payment_map
from .queuing_system import QueuingSystem

from aca_protocols.ac_charging_protocol import (
    CarStartedChargingInfo,
    CarFinishedChargingInfo,
)

from uagents.network import get_ledger
from aca_protocols.ac_payment_protocol import TransactionInfo
from .payment import send_payment_request, confirm_transaction

hostname = socket.gethostname()
IPAddr = socket.gethostbyname(hostname)

agent = Agent(
    name="station",
    seed="Station1",
    port=8001,
    endpoint=["http://{}:8001/submit".format(IPAddr)],
)

fund_agent_if_low(agent.wallet.address(), min_balance=MIN_TEST_AMOUNT)

agent.include(stationRegisterProtocol)

agent.include(carRegisterProtocol)
agent.include(propertyQueryProtocol)

rclpy.init()
minimal_publisher = MinimalPublisher()

fund_agent_if_low(str(agent.wallet.address()))


@agent.on_event("startup")
async def startup_event(ctx: Context):
    ctx.logger.info(f"Agent: {agent.name} ({agent.address})")
    minimal_publisher.log_message(f"Agent: {agent.name} ({agent.address})")

    # run function in background so agent can fully start while registering
    asyncio.ensure_future(register_at_registry(ctx, agent))

    queuing_system = QueuingSystem(reservations=[])

    properties = PropertyData(
        open_time_frames=queuing_system.open_time_frames(),
        geo_point=(20.32, 85.52),
        cost_per_kwh=34.76,
        charging_wattage=11,
        green_energy=False,
    )

    ctx.storage.set("queuing_system", queuing_system.to_json())
    ctx.storage.set("properties", properties.to_json())
    ctx.storage.set("expireAt", datetime.datetime.fromtimestamp(86400).timestamp())

    initialize_payment_map(ctx)


@agent.on_message(model=CarStartedChargingInfo)
async def on_car_started_charging(
    ctx: Context, sender: str, _msg: CarStartedChargingInfo
):
    ctx.logger.info(f"car {sender} charging")


@agent.on_message(model=CarFinishedChargingInfo)
async def on_car_completed_charging(
    ctx: Context, sender: str, _msg: CarFinishedChargingInfo
):
    ctx.logger.info(f"car {sender} finished charging")
    ledger = get_ledger(test=True)
    agent_balance = ledger.query_bank_balance(agent.wallet.address())

    await send_payment_request(
        ctx, sender, _msg, str(agent.wallet.address()), agent_balance
    )


@agent.on_message(model=TransactionInfo)
async def on_transaction_info(ctx: Context, sender: str, _msg: TransactionInfo):
    ctx.logger.info(f"car {sender} sent transaction info: {_msg}")
    ledger = get_ledger(test=True)
    agent_balance = ledger.query_bank_balance(agent.wallet.address())

    await confirm_transaction(
        ctx, sender, _msg, str(agent.wallet.address()), agent_balance
    )


def main(args=None):
    agent.run()


if __name__ == "__main__":
    main()
