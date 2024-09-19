import datetime
import math
import asyncio
import rclpy

import spade
from spade.agent import Agent
from spade.behaviour import CyclicBehaviour
from spade.message import Message

import drone_explore as agent_drone_explore
import rescue as agent_rescue

async def main():
    rescue_agent2 = agent_rescue.RescueAgent("rescue_drone2@jabbers.one", "rescue_drone2")
    rescue_agent2.id = 2
    rescue_agent2.receiver_another = "testesmagia@jabbers.one"

    rclpy.init(args=None)
    
    await rescue_agent2.start()

    await spade.wait_until_finished(rescue_agent2)

    await rescue_agent2.stop()

    print("Agent 2 finished")

if __name__ == "__main__":
    spade.run(main())
