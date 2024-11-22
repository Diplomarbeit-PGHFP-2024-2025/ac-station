from asyncio import sleep

from aca_protocols.station_register_protocol import (
    StationRegisterResponse,
)

from uagents import Context, Protocol

import datetime

from uagents import Agent

from aca_protocols.station_register_protocol import (
    StationRegisterRequest,
)

from aca_protocols.property_query_protocol import (
    PropertyQueryRequest,
    PropertyQueryResponse,
    PropertyData,
)

from aca_protocols.acs_registry_id import acs_id

protocol = Protocol()


@protocol.on_message(model=PropertyQueryRequest, replies=PropertyQueryResponse)
async def on_query_properties(ctx: Context, sender: str, request: PropertyQueryRequest):
    ctx.logger.info(f"{sender} requested params")

    properties = ctx.storage.get("properties")
    properties = PropertyData.from_json(properties)

    response = PropertyQueryResponse(properties=properties)

    await ctx.send(sender, response)


@protocol.on_message(model=StationRegisterResponse)
async def on_is_registered(ctx: Context, sender: str, request: StationRegisterResponse):
    ctx.logger.info(f"got registered by: {sender}; TTL: {request.ttl}")
    ctx.storage.set(
        "expireAt", datetime.datetime.now().timestamp() + (request.ttl * 0.5)
    )


async def register_at_registry(ctx: Context, agent: Agent):
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
