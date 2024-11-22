from aca_protocols.car_register_protocol import CarRegisterRequest, CarRegisterResponse

from uagents import Context, Protocol

protocol = Protocol()


@protocol.on_message(model=CarRegisterRequest, replies=CarRegisterResponse)
async def on_register_car(ctx: Context, sender: str, request: CarRegisterRequest):
    ctx.logger.info(f"car {sender} wants to be registered: {request}")
    await ctx.send(sender, CarRegisterResponse(success=True))
