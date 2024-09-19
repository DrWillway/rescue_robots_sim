import asyncio
import spade
from spade.agent import Agent
from spade.behaviour import CyclicBehaviour, OneShotBehaviour
from spade.message import Message
from spade.template import Template

# Define a global variable for busy state
busy = False

class SenderAgent(Agent):
    class InformBehav(CyclicBehaviour):
        async def run(self):
            global busy  # Access the global busy variable

            print("InformBehav running")
            while True:
                if busy:
                    msg = Message(to=self.agent.recv_jid)
                    msg.set_metadata("performative", "inform")
                    msg.body = "AAAAAAAAA"
                    await self.send(msg)
                    print("Message sent!")
                else:
                    print("Robot is not busy. Waiting...")

                await asyncio.sleep(5)  # Send message every 5 seconds

    async def setup(self):
        print("SenderAgent started")
        b = self.InformBehav()
        self.add_behaviour(b)

    def __init__(self, recv_jid, *args, **kwargs):
        self.recv_jid = recv_jid
        super().__init__(*args, **kwargs)


class RescueRobotAgent(Agent):
    class ReceiveNotificationBehaviour(CyclicBehaviour):
        async def run(self):
            global busy  # Access the global busy variable

            print("Waiting for message...")
            msg = await self.receive(timeout=10)
            if msg:
                print(f"Received message from {msg.sender}: {msg.body}; robot busy: {busy}")
                busy = True  # Set robot to busy upon receiving a message
            else:
                print("No message received within timeout period.")
                busy = False  # Set robot to not busy if no message received

    async def setup(self):
        print("Rescue Robot Agent starting...")
        receive_behaviour = self.ReceiveNotificationBehaviour()
        template = Template()
        template.set_metadata("performative", "inform")
        self.add_behaviour(receive_behaviour)

async def main():
    recv_jid = 'testesmagia@jabbers.one'
    recv_passwd = 'password'  # Replace with the actual password
    sender_jid = 'rescure_drone@jabbers.one'
    sender_passwd = 'rescure_drone'

    # Create and start the RescueRobotAgent
    rescue_robot_agent = RescueRobotAgent(recv_jid, recv_passwd)
    await rescue_robot_agent.start()
    print("Rescue Robot Agent started")

    # Create and start the SenderAgent
    sender_agent = SenderAgent(recv_jid, sender_jid, sender_passwd)
    await sender_agent.start(auto_register=True)
    print("Sender Agent started")

    # Wait until RescueRobotAgent finishes (you can customize this part based on your requirements)
    await spade.wait_until_finished(rescue_robot_agent)
    print("Rescue Robot Agent finished")

    # Stop the SenderAgent (optional)
    await sender_agent.stop()
    print("Sender Agent stopped")

if __name__ == "__main__":
    asyncio.run(main())
