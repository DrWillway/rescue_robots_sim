import spade
from spade.agent import Agent
from spade.behaviour import CyclicBehaviour
from spade.message import Message
from spade.template import Template
from time import sleep

class RescueRobotAgent(Agent):
    class ReceiveNotificationBehaviour(CyclicBehaviour):
        async def run(self):
            print("Waiting for message...")
            msg = await self.receive(timeout=10)  # Wait for a message for 10 seconds
            if msg:
                print(f"Received message from {msg.sender}: {msg.body}")
                # i = 0
                # while i < 10:
                #     print(i)
                #     sleep(1)
                #     i += 1
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
    robot_type = input("Enter robot type (1 for testesmagia@jabbers.one, any other for rescure_drone@jabbers.one): ")

    if robot_type == '1':
        jid = 'testesmagia@jabbers.one'
        passwd = 'password'
    else:
        jid = 'rescure_drone@jabbers.one'
        passwd = 'rescure_drone'

    rescue_robot_agent = RescueRobotAgent(jid, passwd)
    await rescue_robot_agent.start()
    print("Rescue Robot Agent started")

    await spade.wait_until_finished(rescue_robot_agent)
    print("Agents finished")

if __name__ == "__main__":
    spade.run(main())
