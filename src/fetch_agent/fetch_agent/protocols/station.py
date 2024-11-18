from aca_protocols.station_register_protocol import (
    StationRegisterResponse,
)

from uagents import Context, Protocol

import datetime

protocol = Protocol()


@protocol.on_message(model=StationRegisterResponse)
async def on_is_registered(ctx: Context, sender: str, request: StationRegisterResponse):
    ctx.logger.info(f"got registered by: {sender}; TTL: {request.ttl}")
    ctx.storage.set("expireAt", datetime.datetime.now().timestamp() + (request.ttl * 0.5))