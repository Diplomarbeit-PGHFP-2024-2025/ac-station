import asyncio
import datetime
import socket
import rclpy

from .queuing_system import QueuingSystem
from .minimal_publisher import MinimalPublisher

from uagents import Agent, Context
from uagents.setup import fund_agent_if_low

from aca_protocols.property_query_protocol import (
    PropertyData,
)

from .communication.registry import stationRegisterProtocol, register_at_registry
from .communication.car import propertyQueryProtocol, carRegisterProtocol

hostname = socket.gethostname()
IPAddr = socket.gethostbyname(hostname)

agent = Agent(
    name="station",
    seed="Station1",
    port=8001,
    endpoint=["http://{}:8001/submit".format(IPAddr)],
)

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

    queuing_system = QueuingSystem(
        reservations=[]
    )

    properties = PropertyData(
        open_time_frames=[(0, 0), (0, 0)],
        geo_point=(20.32, 85.52),
        cost_per_kwh=34.76,
        charging_wattage=11,
        green_energy=False,
    )

    ctx.storage.set("queuing_system", queuing_system.to_json())
    ctx.storage.set("properties", properties.to_json())
    ctx.storage.set("expireAt", datetime.datetime.fromtimestamp(86400).timestamp())


def main():
    agent.run()


if __name__ == "__main__":
    main()
