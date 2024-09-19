import asyncio
import spade
from spade.agent import Agent
from spade.behaviour import CyclicBehaviour
from spade.behaviour import OneShotBehaviour

class AgentExample(Agent):
    async def setup(self):
        print(f"{self.jid} created.")

class DummyAgent(Agent):
    class MyBehav(CyclicBehaviour):
        async def on_start(self):
            print("Starting behaviour . . .")
            self.counter = 0

        async def run(self):
            print("Counter: {}".format(self.counter))
            self.counter += 1
            if self.counter > 200:
                self.kill(exit_code=10)
                return
            await asyncio.sleep(1)

        async def on_end(self):
            print("Behaviour finished with exit code {}.".format(self.exit_code))
    
    async def setup(self):
        print("Agent starting . . .")
        self.my_behav = self.MyBehav()
        self.add_behaviour(self.my_behav)

class CreateBehav(OneShotBehaviour):
    async def run(self):
        agent2 = AgentExample(jid="testesmagia@jabbers.one", password="password")
        await agent2.start(auto_register=True)

async def main():
    dummy = DummyAgent(jid="c1@jabbers.one", password="teste_123")
    dummy.web.start(hostname="127.0.0.1", port="10000")
    behav = CreateBehav()
    dummy.add_behaviour(behav)
    await dummy.start(auto_register=True)


    # wait until user interrupts with ctrl+C
    while not dummy.my_behav.is_killed():
        try:
            await asyncio.sleep(1)
        except KeyboardInterrupt:
            break

    assert dummy.my_behav.exit_code == 10

    await dummy.stop()


if __name__ == "__main__":
    spade.run(main())