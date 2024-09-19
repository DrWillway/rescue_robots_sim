import getpass

import spade
from spade.agent import Agent
from spade.behaviour import CyclicBehaviour
from spade.message import Message
from spade.template import Template

class RescueRobotAgent(Agent):
    class ReceiveNotificationBehaviour(CyclicBehaviour):
        async def run(self):
            print("Waiting for message...")
            msg = await self.receive(timeout=10)  # Wait for a message for 10 seconds
            if msg:
                print(f"Received message from {msg.sender}: {msg.body}")
                # Here you can add any additional handling of the message
            else:
                print("No message received within timeout period.")

    async def setup(self):
        print("Rescue Robot Agent starting...")
        receive_behaviour = self.ReceiveNotificationBehaviour()        
        template = Template()
        template.set_metadata("performative", "inform")
        self.add_behaviour(receive_behaviour)


async def main():
    jid = 'testesmagia@jabbers.one'
    passwd = 'password'  # Replace with the actual password
    rescue_robot_agent = RescueRobotAgent(jid, passwd)
    await rescue_robot_agent.start()
    print("Rescue Robot Agent started")

    await spade.wait_until_finished(rescue_robot_agent)
    print("Agents finished")

if __name__ == "__main__":
    spade.run(main())
