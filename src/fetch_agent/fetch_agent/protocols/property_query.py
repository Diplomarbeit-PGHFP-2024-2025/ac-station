from aca_protocols.property_query_protocol import (
    PropertyQueryRequest,
    PropertyQueryResponse,
    PropertyData
)

from uagents import Context, Protocol

protocol = Protocol()


@protocol.on_message(model=PropertyQueryRequest, replies=PropertyQueryResponse)
async def on_query_properties(ctx: Context, sender: str, request: PropertyQueryRequest):
    ctx.logger.info(f"{sender} requested params")

    properties = ctx.storage.get("properties")
    properties = PropertyData.from_json(properties)

    response = PropertyQueryResponse(properties=properties)

    await ctx.send(sender, response)
