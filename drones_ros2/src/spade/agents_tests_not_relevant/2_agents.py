import datetime
import math
import asyncio
import spade
from spade.agent import Agent
from spade.behaviour import CyclicBehaviour
from spade.message import Message

target_x, target_y = 50, 50  # Define the point (x, y)
robot1_position = (10, 10)
robot2_position = (60, 60)

class SenderAgent(Agent):
    class InformBehav(CyclicBehaviour):
        async def run(self):
            # print(f"Sender {self.agent.jid} running at {datetime.datetime.now().time()}")
            position = self.agent.position
            distance = math.sqrt((position[0] - target_x)**2 + (position[1] - target_y)**2)
            
            msg = Message(to=self.agent.receiver_jid)
            msg.body = str(distance)
            await self.send(msg)
            print(f"Sender {self.agent.id} sent distance: {distance}")

            await self.agent.stop()

    async def setup(self):
        # self.id = 0
        print(f"SenderAgent {self.id} started with position {self.position}")
        b = self.InformBehav()
        self.add_behaviour(b)


class ReceiverAgent(Agent):
    class RecvBehav(CyclicBehaviour):
        async def run(self):
            # print(f"Receiver {self.agent.jid} running")
            msg = await self.receive(timeout=10)
            if msg:
                distance = float(msg.body)
                self.agent.distances.append(distance)
                print(f"Receiver {self.agent.id} received distance: {distance}")

                if len(self.agent.distances) == 2:
                    d1, d2 = self.agent.distances
                    if d1 < d2:
                        print(f"Robot 1 is closer with distance {d1}")
                    else:
                        print(f"Robot 2 is closer with distance {d2}")
                    await self.agent.stop()
            else:
                print(f"Receiver {self.agent.id} did not receive any message after 10 seconds")
                await self.agent.stop()

    async def setup(self):
        # self.id = 0
        self.distances = []
        print(f"ReceiverAgent {self.id} started")
        b = self.RecvBehav()
        self.add_behaviour(b)


async def main():
    sender1 = SenderAgent("c1@jabbers.one", "teste_123")
    sender1.position = robot1_position
    sender1.id = 1
    sender1.receiver_jid = "testesmagia@jabbers.one"
    await sender1.start(auto_register=True)
    
    sender2 = SenderAgent("rescure_drone@jabbers.one", "rescure_drone")
    sender2.position = robot2_position
    sender2.id = 2
    sender2.receiver_jid = "rescue_drone2@jabbers.one"
    await sender2.start(auto_register=True)
    
    receiver1 = ReceiverAgent("testesmagia@jabbers.one", "password")
    receiver1.id = 1
    await receiver1.start(auto_register=True)

    receiver2 = ReceiverAgent("rescue_drone2@jabbers.one", "rescue_drone2")
    receiver2.id = 2
    await receiver2.start(auto_register=True)

    # Optionally, start the web interfaces for monitoring
    sender1.web.start(hostname="127.0.0.1", port="10000")
    sender2.web.start(hostname="127.0.0.1", port="10001")
    receiver1.web.start(hostname="127.0.0.1", port="10002")
    receiver2.web.start(hostname="127.0.0.1", port="10003")

    await spade.wait_until_finished(receiver1)
    await spade.wait_until_finished(receiver2)

    await sender1.stop()
    await sender2.stop()
    await receiver1.stop()
    await receiver2.stop()
    print("Agents finished")


if __name__ == "__main__":
    asyncio.run(main())
