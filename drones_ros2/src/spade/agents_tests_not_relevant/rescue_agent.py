import spade
from spade.agent import Agent
from spade.behaviour import OneShotBehaviour
from spade.message import Message
from spade.template import Template


class SenderAgent(Agent):
    class InformBehav(OneShotBehaviour):
        async def run(self):
            print("InformBehav running")
            msg = Message(to=self.agent.recv_jid)
            msg.set_metadata("performative", "inform")
            msg.body = f"Hello from SenderAgent {self.agent.name}"
            await self.send(msg)
            print("Message sent!")
            await self.agent.stop()

    async def setup(self):
        print(f"SenderAgent started")
        b = self.InformBehav()
        self.add_behaviour(b)

    def __init__(self, recv_jid, *args, **kwargs):
        self.recv_jid = recv_jid
        super().__init__(*args, **kwargs)


class ReceiverAgent(Agent):
    class RecvBehav(OneShotBehaviour):
        async def run(self):
            print("RecvBehav running")
            msg = await self.receive(timeout=10)
            if msg:
                print(f"Message received with content: {msg.body}")
            else:
                print("Did not receive any message after 10 seconds")
            await self.agent.stop()

    async def setup(self):
        print(f"ReceiverAgent {self.name} started")
        b = self.RecvBehav()
        template = Template()
        template.set_metadata("performative", "inform")
        self.add_behaviour(b, template)


async def main():
    sender1_jid = 'testesmagias@jabbers.one'
    sender1_passwd = 'password'

    sender2_jid = 'c1s@jabbers.one'
    sender2_passwd = 'teste_123'

    receiver1_jid = 'rescure_drones@jabbers.one'
    receiver1_passwd = 'rescure_drone'

    receiver2_jid = 'drwillways@jabbers.one'
    receiver2_passwd = 'galaxyeyes9!'

    receiveragent1 = ReceiverAgent("ReceiverAgent1", receiver1_jid, receiver1_passwd)
    receiveragent2 = ReceiverAgent("ReceiverAgent2", receiver2_jid, receiver2_passwd)
    senderagent1 = SenderAgent("SenderAgent1", receiver1_jid, sender1_jid, sender1_passwd)
    senderagent2 = SenderAgent("SenderAgent2", receiver2_jid, sender2_jid, sender2_passwd)
   

    try:
        await senderagent1.start(auto_register=True)
        await senderagent2.start(auto_register=True)
        await receiveragent1.start(auto_register=True)
        await receiveragent2.start(auto_register=True)

        print("Agents started")

        await spade.wait_until_finished(senderagent1)
        await spade.wait_until_finished(senderagent2)
        await spade.wait_until_finished(receiveragent1)
        await spade.wait_until_finished(receiveragent2)

        print("Agents finished")

    except Exception as e:
        print(f"Exception occurred: {e}")

if __name__ == "__main__":
    spade.run(main())
