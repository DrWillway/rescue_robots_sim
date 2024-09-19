import random
import asyncio
import spade
from spade import wait_until_finished
from spade.agent import Agent
from spade.behaviour import FSMBehaviour, State
from spade.message import Message
from time import sleep 

STATE_ONE = "START"
STATE_TWO = "EXPLORE"
STATE_THREE = "HUMAN_FOUND"
STATE_FOUR = "INFORM_RESCUE_ROBOT"
STATE_FIVE = "FINISH"


class DroneFSMBehaviour(FSMBehaviour):
    async def on_start(self):
        print(f"FSM starting at initial state {self.current_state}")

    async def on_end(self):
        print(f"FSM finished at state {self.current_state}")
        await self.agent.stop()


class StateOne(State):
    async def run(self):
        print("I'm at state START (initial state)")
        self.set_next_state(STATE_TWO)


class StateTwo(State):
    async def run(self):
        print("I'm at state EXPLORE")

        # Initialize or retrieve the counter from agent memory
        sleep(1)
        if "counter" not in self.agent.memory:
            self.agent.memory["counter"] = 0
        else:
            self.agent.memory["counter"] += 1

        counter = self.agent.memory["counter"]
        print(f"Exploration cycle {counter}")

        if counter > 20:
            self.set_next_state(STATE_FIVE)
            return
        
        # Simulate the exploration and finding a human (randomly for demo purpose)
        human_found = random.choice([True, False])

        if human_found:
            self.set_next_state(STATE_THREE)
        else:
            self.set_next_state(STATE_TWO)  # Continue exploring


class StateThree(State):
    async def run(self):
        print("I'm at state HUMAN_FOUND")
        # Here, you can handle any immediate processing when a human is found
        self.set_next_state(STATE_FOUR)


class StateFour(State):
    async def run(self):
        print("I'm at state INFORM_RESCUE_ROBOT")
        # Notify the rescue robot agent
        rescue_robot1_jid = "testesmagia@jabbers.one"
        msg = Message(to=rescue_robot1_jid)
        msg.set_metadata(
            "performative", "inform"
        )  # Set the "inform" FIPA performative
        msg.body = "10"
        await self.send(msg)
        print(f"Notification sent to {rescue_robot1_jid}")
        rescue_robot2_jid = "rescure_drone@jabbers.one"
        msg = Message(to=rescue_robot2_jid)
        msg.set_metadata(
            "performative", "inform"
        )  # Set the "inform" FIPA performative
        msg.body = "10"
        await self.send(msg)
        print(f"Notification sent to {rescue_robot2_jid}")
        self.set_next_state(STATE_TWO)  # After notifying, go back to exploration


class StateFive(State):
    async def run(self):
        print("I'm at state FINISH (final state)")
        # Final cleanup or message handling if needed
        msg = await self.receive(timeout=5)
        if msg:
            print(f"State Five received message {msg.body}")
        # No further state as this is the final state


class FSMAgent(Agent):
    async def setup(self):
        self.memory = {}  # Initialize memory as an empty dictionary
        self.fsm = DroneFSMBehaviour()
        self.fsm.add_state(name=STATE_ONE, state=StateOne(), initial=True)
        self.fsm.add_state(name=STATE_TWO, state=StateTwo())
        self.fsm.add_state(name=STATE_THREE, state=StateThree())
        self.fsm.add_state(name=STATE_FOUR, state=StateFour())
        self.fsm.add_state(name=STATE_FIVE, state=StateFive())
        self.fsm.add_transition(source=STATE_ONE, dest=STATE_TWO)
        self.fsm.add_transition(source=STATE_TWO, dest=STATE_THREE)
        self.fsm.add_transition(source=STATE_TWO, dest=STATE_TWO)
        self.fsm.add_transition(source=STATE_THREE, dest=STATE_FOUR)
        self.fsm.add_transition(source=STATE_FOUR, dest=STATE_TWO)  # Return to explore after informing
        self.fsm.add_transition(source=STATE_TWO, dest=STATE_FIVE)  # Finish exploring if necessary
        self.add_behaviour(self.fsm)


async def main():
    jid = 'c1@jabbers.one'
    passwd = 'teste_123'
    fsmagent = FSMAgent(jid, passwd)
    await fsmagent.start()

    # wait until user interrupts with ctrl+C
    while not fsmagent.fsm.is_killed():
        try:
            await asyncio.sleep(1)
        except KeyboardInterrupt:
            break

    await fsmagent.stop()

    print("Agent finished")

if __name__ == "__main__":
    spade.run(main())
