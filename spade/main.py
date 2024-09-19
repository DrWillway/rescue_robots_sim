import spade
from flask import Flask

from agents.ReceiverAgent import ReceiverAgent
from utils.communication import PeriodicSenderAgent

app = Flask(__name__)


async def main():

    receiveragent = ReceiverAgent(jid="testesmagia@jabbers.one", password="password")
    await receiveragent.start(auto_register=True)
    print("Receiver started")

    senderagent = PeriodicSenderAgent(jid="wheelsofwisdom1@jabbers.one", password="wheelsofwisdom1")
    senderagent.set("receiver_jid", "testesmagia@jabbers.one")
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

