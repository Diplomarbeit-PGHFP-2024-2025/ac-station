import asyncio
from asyncio import sleep
import datetime
import socket

from uagents import Agent, Context
from uagents.setup import fund_agent_if_low

from aca_protocols.station_register_protocol import (
    StationRegisterResponse,
    StationRegisterRequest,
)

from aca_protocols.acs_registry_id import acs_id

hostname = socket.gethostname()
IPAddr = socket.gethostbyname(hostname)

agent = Agent(
    name="station",
    seed="Station1",
    port=8001,
    endpoint=["http://{}:8001/submit".format(IPAddr)],
)

fund_agent_if_low(agent.wallet.address())


@agent.on_event("startup")
async def startup_event(ctx: Context):
    ctx.logger.info(f"Agent: {agent.name} ({agent.address})")

    # run function in background so agent can fully start while registering
    asyncio.ensure_future(register_at_registry(ctx))


async def register_at_registry(ctx: Context):
    ctx.storage.set("expireAt", datetime.datetime.fromtimestamp(86400).timestamp())
    while (
            datetime.datetime.fromtimestamp(ctx.storage.get("expireAt"))
            < datetime.datetime.now()
    ):
        ctx.logger.info(f"Trying to introduce: {agent.name} ({agent.address})")
        await ctx.send(
            acs_id,
            StationRegisterRequest(lat=1.0, long=1.0),
        )

        await sleep(6)


@agent.on_message(StationRegisterResponse)
async def on_is_registered(ctx: Context, sender: str, _msg: StationRegisterResponse):
    ctx.logger.info(f"got registered by: {sender}; TTL: {_msg.ttl}")
    ctx.storage.set("expireAt", datetime.datetime.now().timestamp() + (_msg.ttl * 0.5))
def main(args=None):
    agent.run()


if __name__ == "__main__":
    main()
