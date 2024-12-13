from aca_protocols.car_register_protocol import CarRegisterRequest, CarRegisterResponse

from uagents import Context, Protocol

from aca_protocols.property_query_protocol import (
    PropertyQueryRequest,
    PropertyQueryResponse,
    PropertyData,
)

from aca_protocols.ac_cancel_timeframe import (
    CancelTimeFrameResponse,
    CancelTimeFrameReqeust,
)

from ..queuing_system import QueuingSystem

carRegisterProtocol = Protocol()


@carRegisterProtocol.on_message(model=CarRegisterRequest, replies=CarRegisterResponse)
async def on_register_car(ctx: Context, sender: str, request: CarRegisterRequest):
    ctx.logger.info(f"car {sender} wants to be registered: {request}")

    queuing_system_serialized = ctx.storage.get("queuing_system")
    queuing_system: QueuingSystem = QueuingSystem.from_json(queuing_system_serialized)

    if queuing_system.does_conflict(int(request.start_time), int(request.duration)):
        await ctx.send(sender, CarRegisterResponse(success=False))
        return

    properties = ctx.storage.get("properties")
    properties = PropertyData.from_json(properties)

    queuing_system.add_timeframe_for_sender(
        sender, (int(request.start_time), int(request.start_time + request.duration))
    )
    properties.open_time_frames = queuing_system.open_time_frames()
    ctx.storage.set("properties", properties.to_json())
    ctx.storage.set("queuing_system", queuing_system.to_json())

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


cancellationProtocol = Protocol()


@cancellationProtocol.on_message(
    model=CancelTimeFrameReqeust, replies=CancelTimeFrameResponse
)
async def on_time_frame_cancellation(
    ctx: Context, sender: str, request: CancelTimeFrameReqeust
):
    ctx.logger.info(f"{sender} requested timeframe cancellation")
    queuing_system_serialized = ctx.storage.get("queuing_system")
    queuing_system: QueuingSystem = QueuingSystem.from_json(queuing_system_serialized)

    queuing_system.remove_timeframe_for_sender(
        request.sender_address, request.timeframe
    )

    properties_serialized = ctx.storage.get("properties")
    properties = PropertyData.from_json(properties_serialized)
    properties.open_time_frames = queuing_system.open_time_frames()
    ctx.storage.set("properties", properties.to_json())

    ctx.storage.set("queuing_system", queuing_system.to_json())

    await ctx.send(sender, CancelTimeFrameResponse())
