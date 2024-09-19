import asyncio
import spade
from spade.agent import Agent
from spade.behaviour import CyclicBehaviour
from spade.message import Message
import math 

distances = {}
robot1_position = (10.0, 10.0)
robot2_position = (60.0, 60.0)

class ReceiverAgent(Agent):
    class RecvBehav(CyclicBehaviour):
        async def run(self):
            global distances
            msg = await self.receive(timeout=10)
            if msg:
                target_str = msg.body
                target = eval(target_str) 
                sender_jid = msg.sender
                sender_id = str(sender_jid.localpart)  # Extract the username (agent id) from JID
                position = self.agent.position
                distance = math.sqrt((position[0] - float(target[0]))**2 + (position[1] - float(target[1]))**2)

                distances[sender_id] = distance
                print(f"Receiver {self.agent.id} received distance {distance} from {sender_id}")

                if len(distances) == 2:
                    d1 = distances["c1"]
                    d2 = distances["rescure_drone"]
                    
                    if d1 < d2:
                        print(f"Robot 1 is closer with distance {d1}")
                        if self.agent.id == 1:
                            await self.navigate_to_human(1)
                    else:
                        print(f"Robot 2 is closer with distance {d2}")
                        if self.agent.id == 2:
                            await self.navigate_to_human(2)
                    
                    distances.clear()  # Clear the distances after comparison
            else:
                print(f"Receiver {self.agent.id} did not receive any message after 10 seconds")

        async def navigate_to_human(self, id):
            print(f"I am closer, id: {id}")
    
    async def setup(self):
        print(f"ReceiverAgent {self.id} started")
        b = self.RecvBehav()
        self.add_behaviour(b)

async def main():
    receiver1 = ReceiverAgent("testesmagia@jabbers.one", "password")
    receiver1.id = 1
    receiver1.position = robot1_position
    await receiver1.start(auto_register=True)

    receiver2 = ReceiverAgent("rescue_drone2@jabbers.one", "rescue_drone2")
    receiver2.id = 2
    receiver2.position = robot2_position
    await receiver2.start(auto_register=True)

    # Optionally, start the web interfaces for monitoring
    receiver1.web.start(hostname="127.0.0.1", port="10002")
    receiver2.web.start(hostname="127.0.0.1", port="10003")

    await spade.wait_until_finished(receiver1)
    await receiver1.stop()
    await receiver2.stop()
    
    print("Agents finished")

if __name__ == "__main__":
    spade.run(main())
