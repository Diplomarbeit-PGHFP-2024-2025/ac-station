from uagents import Agent, Context
import socket
import rclpy

from .minimal_publisher import MinimalPublisher

class MyRosAgent(Agent):

    def __init__(self, name, port, seed, endpoint, publisher=None, listener=None):
        super().__init__(name, port, seed, endpoint)
        self.publisher = publisher
        self.listener = listener

    def get_listener(self):
        return self.listener

    def get_publisher(self):
        return self.publisher


hostname = socket.gethostname()
IPAddr = socket.gethostbyname(hostname)

rclpy.init()

minimal_publisher = MinimalPublisher()

agent = MyRosAgent(
    name="station",
    seed="Station1",
    port=8001,
    endpoint=["http://{}:8001/submit".format(IPAddr)],
    publisher=minimal_publisher
)

@agent.on_event("startup")
async def startup_event(ctx: Context):
    ctx.logger.info(f"Agent: {agent.name} ({agent.address})")
    agent.get_publisher().log_message(f"Agent: {agent.name} ({agent.address})")


def main(args=None):
    agent.run()


if __name__ == "__main__":
    main()