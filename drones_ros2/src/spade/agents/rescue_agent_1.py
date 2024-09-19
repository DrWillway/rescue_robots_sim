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
    rescue_agent1 = agent_rescue.RescueAgent("testesmagia@jabbers.one", "password")
    rescue_agent1.id = 1
    rescue_agent1.receiver_another = "rescue_drone2@jabbers.one"

    rclpy.init(args=None)
           
    await rescue_agent1.start()

    await spade.wait_until_finished(rescue_agent1)

    await rescue_agent1.stop()

    print("Agent 1 finished")

if __name__ == "__main__":
    spade.run(main())
