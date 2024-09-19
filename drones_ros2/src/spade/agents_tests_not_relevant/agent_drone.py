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
            msg = Message(to=self.agent.receiver_jid1)
            msg.body = str((target_x,target_y, 1))
            await self.send(msg)
            msg = Message(to=self.agent.receiver_jid2)
            msg.body = str((target_x,target_y, 2))
            await self.send(msg)
            print(f"Drone {self.agent.id} sent human location: {(target_x,target_y)}")

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
        print(f"UAV drone {self.id} started")
        start_at = datetime.datetime.now() + datetime.timedelta(seconds=5)
        b = self.InformBehav(period=2, start_at=start_at)
        self.add_behaviour(b)