from uagents import Context
from aca_protocols.ac_payment_protocol import PaymentRequest, TransactionInfo, DENOM
from aca_protocols.ac_charging_protocol import CarFinishedChargingInfo
from aca_protocols.property_query_protocol import (
    PropertyData,
)
from uagents.network import wait_for_tx_to_complete


def initialize_payment_map(ctx: Context):
    to_set: {str, float} = {}

    ctx.storage.set("expected_payment", to_set)


async def send_payment_request(
    ctx: Context,
    car_address: str,
    info: CarFinishedChargingInfo,
    agent_wallet_address: str,
    agent_balance: int,
):
    properties: PropertyData = PropertyData.from_json(ctx.storage.get("properties"))

    amount_to_pay: int = int(round(properties.cost_per_kwh * info.kwh_charged, 0))

    ctx.logger.info(
        f"[Payment, send_payment_request]: Sending payment request {car_address} for amount {amount_to_pay}. Current balance: {agent_balance}"
    )

    await ctx.send(
        car_address,
        PaymentRequest(
            wallet_address=agent_wallet_address,
            amount=amount_to_pay,
            denomination=DENOM,
        ),
    )

    expected_payment: {str, float} = ctx.storage.get("expected_payment")
    expected_payment[car_address] = amount_to_pay
    ctx.storage.set("expected_payment", expected_payment)


async def confirm_transaction(
    ctx: Context,
    car_address: str,
    info: TransactionInfo,
    agent_wallet_address: str,
    agent_balance: int,
):
    ctx.logger.info(
        f"[Payment, confirm_transaction]: Received transaction info from {car_address}: {info}"
    )
    tx_resp = await wait_for_tx_to_complete(info.transaction_hash, ctx.ledger)

    coin_received = tx_resp.events["coin_received"]
    expected_amount = ctx.storage.get("expected_payment")[car_address]

    if (
        coin_received["receiver"] == agent_wallet_address
        and coin_received["amount"] == f"{expected_amount}{DENOM}"
    ):
        ctx.logger.info(
            f"[Payment, confirm_transaction]: Transaction successful: {car_address} payed {expected_amount}{DENOM}. New balance: {agent_balance}"
        )
