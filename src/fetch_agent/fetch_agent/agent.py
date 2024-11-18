import asyncio
from asyncio import sleep
import datetime
import socket
import rclpy

from .minimal_publisher import MinimalPublisher

from uagents import Agent, Context
from uagents.setup import fund_agent_if_low

from aca_protocols.station_register_protocol import (
    StationRegisterRequest,
)

from aca_protocols.property_query_protocol import (
    PropertyData,
)

from aca_protocols.acs_registry_id import acs_id

from .protocols.station import protocol as station_protocol
from .protocols.car import protocol as car_protocol
from .protocols.property_query import protocol as property_query_protocols

hostname = socket.gethostname()
IPAddr = socket.gethostbyname(hostname)

agent = Agent(
    name="station",
    seed="Station1",
    port=8001,
    endpoint=["http://{}:8001/submit".format(IPAddr)],
)

agent.include(station_protocol)
agent.include(car_protocol)
agent.include(property_query_protocols)

rclpy.init()
minimal_publisher = MinimalPublisher()

fund_agent_if_low(str(agent.wallet.address()))


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


def main():
    agent.run()


if __name__ == "__main__":
    main()
