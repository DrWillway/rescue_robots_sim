import datetime
import math
import asyncio
from time import sleep

import spade
from spade.agent import Agent
from spade.behaviour import PeriodicBehaviour
from spade.message import Message

target_x, target_y = 50, 50  # Define the point (x, y)

class SenderAgent(Agent):
    class InformBehav(PeriodicBehaviour):
        async def run(self):
            global target_x, target_y
            # print(f"Sender {self.agent.jid} running at {datetime.datetime.now().time()}")
            msg = Message(to=self.agent.receiver_jid)
            msg.body = str((target_x,target_y))
            await self.send(msg)
            print(f"Sender {self.agent.id} sent target: {(target_x,target_y)}")

            if self.counter == 10:
                self.kill()
            sleep(1)
            self.counter += 1

        async def on_end(self):
            # stop agent from behaviour
            await self.agent.stop()

        async def on_start(self):
            self.counter = 0

    async def setup(self):
        print(f"SenderAgent {self.id} started")
        start_at = datetime.datetime.now() + datetime.timedelta(seconds=5)
        b = self.InformBehav(period=2, start_at=start_at)
        self.add_behaviour(b)

async def main():
    sender1 = SenderAgent("c1@jabbers.one", "teste_123")
    sender1.id = 1
    sender1.receiver_jid = "testesmagia@jabbers.one"
    await sender1.start(auto_register=True)
    
    sender2 = SenderAgent("rescure_drone@jabbers.one", "rescure_drone")
    sender2.id = 2
    sender2.receiver_jid = "rescue_drone2@jabbers.one"
    await sender2.start(auto_register=True)
    
    # Optionally, start the web interfaces for monitoring
    sender1.web.start(hostname="127.0.0.1", port="10000")
    sender2.web.start(hostname="127.0.0.1", port="10001")

    await spade.wait_until_finished(sender1)
    await sender1.stop()
    await sender2.stop()
    print("Agents finished")


if __name__ == "__main__":
    spade.run(main())