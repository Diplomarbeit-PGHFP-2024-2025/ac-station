from aca_protocols.car_register_protocol import CarRegisterRequest, CarRegisterResponse

from uagents import Context, Protocol

from aca_protocols.property_query_protocol import (
    PropertyQueryRequest,
    PropertyQueryResponse,
    PropertyData,
)

from ..queuing_system import QueuingSystem

carRegisterProtocol = Protocol()


@carRegisterProtocol.on_message(model=CarRegisterRequest, replies=CarRegisterResponse)
async def on_register_car(ctx: Context, sender: str, request: CarRegisterRequest):
    ctx.logger.info(f"car {sender} wants to be registered: {request}")

    queuing_system = ctx.storage.get("queuing_system")
    queuing_system = QueuingSystem.from_json(queuing_system)

    if queuing_system.does_conflict(request.start_time, request.duration):
        await ctx.send(sender, CarRegisterResponse(success=False))
        return

    properties = ctx.storage.get("properties")
    properties = PropertyData.from_json(properties)

    queuing_system.reservations.append(
        (request.start_time, request.start_time + request.duration)
    )
    properties.open_time_frames = queuing_system.open_time_frames()
    ctx.storage.set("properties", properties.to_json())

    await ctx.send(sender, CarRegisterResponse(success=True))


propertyQueryProtocol = Protocol()


@propertyQueryProtocol.on_message(
    model=PropertyQueryRequest, replies=PropertyQueryResponse
)
async def on_query_properties(ctx: Context, sender: str, request: PropertyQueryRequest):
    ctx.logger.info(f"{sender} requested params")

    properties = ctx.storage.get("properties")
    response = PropertyQueryResponse(properties=properties)

    await ctx.send(sender, response)
