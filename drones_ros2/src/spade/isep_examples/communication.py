import datetime
import getpass

import spade
from spade.agent import Agent
from spade.behaviour import CyclicBehaviour, PeriodicBehaviour
from spade.message import Message


class PeriodicSenderAgent(Agent):
    class InformBehav(PeriodicBehaviour):
        async def run(self):
            print(f"PeriodicSenderBehaviour running at {datetime.datetime.now().time()}: {self.counter}")
            msg = Message(to=self.get("receiver_jid"))  # Instantiate the message
            msg.body = "Hello World"  # Set the message content

            await self.send(msg)
            print("Message sent!")

            if self.counter == 5:
                self.kill()
            self.counter += 1

        async def on_end(self):
            # stop agent from behaviour
            await self.agent.stop()

        async def on_start(self):
            self.counter = 0

    async def setup(self):
        print(f"PeriodicSenderAgent started at {datetime.datetime.now().time()}")
        start_at = datetime.datetime.now() + datetime.timedelta(seconds=5)
        b = self.InformBehav(period=2, start_at=start_at)
        self.add_behaviour(b)


class ReceiverAgent(Agent):
    class RecvBehav(CyclicBehaviour):
        async def run(self):
            print("RecvBehav running")
            msg = await self.receive(timeout=10)  # wait for a message for 10 seconds
            if msg:
                print("Message received with content: {}".format(msg.body))
            else:
                print("Did not received any message after 10 seconds")
                self.kill()

        async def on_end(self):
            await self.agent.stop()

    async def setup(self):
        print("ReceiverAgent started")
        b = self.RecvBehav()
        self.add_behaviour(b)

async def main():
    # one
    jid = 'c1@jabbers.one'
    passwd = 'teste_123'

    # send and receive
    receiver_jid = 'c1@jabbers.one'
    passwd = 'teste_123'

    sender_jid = 'testesmagia@jabbers.one'
    passwd = 'password'


    receiveragent = ReceiverAgent(jid="c1@jabbers.one", password="teste_123")
    await receiveragent.start(auto_register=True)
    print("Receiver started")

    senderagent = PeriodicSenderAgent(jid="testesmagia@jabbers.one", password="password")
    # senderagent = PeriodicSenderAgent(jid="drwillway@jabbers.one", password="galaxyeyes9!")
    senderagent.set("receiver_jid", "c1@jabbers.one")
    await senderagent.start(auto_register=True)
    print("Sender started")

    senderagent.web.start(hostname="127.0.0.1", port="10000")
    receiveragent.web.start(hostname="127.0.0.1", port="10001")

    await spade.wait_until_finished(receiveragent)
    await senderagent.stop()
    await receiveragent.stop()
    print("Agents finished")


if __name__ == "__main__":
    spade.run(main())
























# import spade









# from spade.agent import Agent
# from spade.behaviour import OneShotBehaviour
# from spade.behaviour import CyclicBehaviour
# from spade.message import Message
# from spade.template import Template


# class SenderAgent(Agent):
#     class InformBehav(CyclicBehaviour):
#         async def run(self):
#             print("InformBehav running")
#             msg = Message(to="receiver@your_xmpp_server")     # Instantiate the message
#             msg.set_metadata("performative", "inform")  # Set the "inform" FIPA performative
#             msg.body = "Hello World"                    # Set the message content

#             await self.send(msg)
#             print("Message sent!")

#             # stop agent from behaviour
#             await self.agent.stop()

#     async def setup(self):
#         print("SenderAgent started")
#         b = self.InformBehav()
#         self.add_behaviour(b)

# class ReceiverAgent(Agent):
#     class RecvBehav(CyclicBehaviour):
#         async def run(self):
#             print("RecvBehav running")

#             msg = await self.receive(timeout=100) # wait for a message for 10 seconds
#             if msg:
#                 print("Message received with content: {}".format(msg.body))
#             else:
#                 print("Did not received any message after 10 seconds")

#             # stop agent from behaviour
#             await self.agent.stop()

#     async def setup(self):
#         print("ReceiverAgent started")
#         b = self.RecvBehav()
#         template = Template()
#         template.set_metadata("performative", "inform")
#         self.add_behaviour(b, template)



# async def main():
#     receiveragent = ReceiverAgent(jid="c1@jabbers.one", password="teste_123")
#     await receiveragent.start(auto_register=True)
#     print("Receiver started")

#     senderagent = SenderAgent(jid="testesmagia@jabbers.one", password="password")
#     await senderagent.start(auto_register=True)
#     print("Sender started")

#     senderagent.web.start(hostname="127.0.0.1", port="10000")
#     receiveragent.web.start(hostname="127.0.0.1", port="10001")

#     await spade.wait_until_finished(receiveragent)
#     print("Agents finished")


# if __name__ == "__main__":
#     spade.run(main())