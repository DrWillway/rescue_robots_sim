import datetime
import math
import asyncio
import spade
from spade.agent import Agent
from spade.behaviour import CyclicBehaviour
from spade.message import Message

import drone_explore as agent_drone_explore
import rescue_receive as agent_rescue

robot1_position = (10.0, 10.0)
robot2_position = (60.0, 60.0)

async def main():
    rescue_robot1 = agent_rescue.ReceiverAgent("testesmagia@jabbers.one", "password")
    rescue_robot1.id = 1
    rescue_robot1.position = robot1_position
    await rescue_robot1.start(auto_register=True)

    rescue_robot2 = agent_rescue.ReceiverAgent("rescue_drone2@jabbers.one", "rescue_drone2")
    rescue_robot2.id = 2
    rescue_robot2.position = robot2_position
    await rescue_robot2.start(auto_register=True)

    rescue_communicate1 = agent_rescue.SenderAgent("rescue_communicate1@jabbers.one", "rescue_communicate1")
    rescue_communicate1.id = 1
    rescue_communicate1.receiver_another = "rescue_drone2@jabbers.one"
    await rescue_communicate1.start(auto_register=True)

    rescue_communicate2 = agent_rescue.SenderAgent("rescue_communicate2@jabbers.one", "rescue_communicate2")
    rescue_communicate2.id = 2
    rescue_communicate2.receiver_another = "testesmagia@jabbers.one"
    await rescue_communicate2.start(auto_register=True)

    # drone1 = agent_drone_explore.FSMAgent("c1@jabbers.one", "teste_123")
    # drone1.id = 1
    # drone1.target_x = 50
    # drone1.target_y = 50
    # drone1.receiver_jid1 = "testesmagia@jabbers.one"
    # drone1.receiver_jid2 = "rescue_drone2@jabbers.one"
    # await drone1.start(auto_register=True)
    
    drone2 = agent_drone_explore.FSMAgent("rescure_drone@jabbers.one", "rescure_drone")
    drone2.id = 2
    drone2.target_x = 20
    drone2.target_y = 30
    drone2.receiver_jid1 = "testesmagia@jabbers.one"
    drone2.receiver_jid2 = "rescue_drone2@jabbers.one"
    await drone2.start(auto_register=True)
    
    # Optionally, start the web interfaces for monitoring
    rescue_robot1.web.start(hostname="127.0.0.1", port="10002")
    rescue_robot2.web.start(hostname="127.0.0.1", port="10003")

    while not drone2.fsm.is_killed():
        try:
            await asyncio.sleep(1)
        except KeyboardInterrupt:
            break

    # await spade.wait_until_finished(rescue_robot1)
    await rescue_robot1.stop()
    await rescue_robot2.stop()

    await spade.wait_until_finished(drone2)
    await drone2.stop()

    
    print("Agents finished")


if __name__ == "__main__":
    spade.run(main())