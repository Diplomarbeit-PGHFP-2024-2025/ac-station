from aca_protocols.car_register_protocol import CarRegisterRequest, CarRegisterResponse

from uagents import Context, Protocol

from aca_protocols.property_query_protocol import (
    PropertyQueryRequest,
    PropertyQueryResponse,
    PropertyData,
)

carRegisterProtocol = Protocol()


@carRegisterProtocol.on_message(model=CarRegisterRequest, replies=CarRegisterResponse)
async def on_register_car(ctx: Context, sender: str, request: CarRegisterRequest):
    ctx.logger.info(f"car {sender} wants to be registered: {request}")
    await ctx.send(sender, CarRegisterResponse(success=True))


propertyQueryProtocol = Protocol()


@propertyQueryProtocol.on_message(
    model=PropertyQueryRequest, replies=PropertyQueryResponse
)
async def on_query_properties(ctx: Context, sender: str, request: PropertyQueryRequest):
    ctx.logger.info(f"{sender} requested params")

    properties = ctx.storage.get("properties")
    properties = PropertyData.from_json(properties)

    response = PropertyQueryResponse(properties=properties)

    await ctx.send(sender, response)
