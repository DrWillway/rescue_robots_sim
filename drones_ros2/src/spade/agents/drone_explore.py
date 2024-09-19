
# spade libs
import spade
from spade.agent import Agent
from spade.behaviour import FSMBehaviour, State
from spade.message import Message

# ros libs
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Point
import threading

# other libs
from time import sleep 
from termcolor import colored
import asyncio

namespace = "drone"

human_found = False
target_x = 0
target_y = 0

class DetectedHumanPositionSubscriber(Node):
    def __init__(self):
        super().__init__('detected_human_pose_subscriber')
        global namespace
        self.human_detected_pose_subscription = self.create_subscription(
            Point,
            f'/{namespace}/detected_pose',
            self.detected_pose_callback,
            10
        )

    def detected_pose_callback(self, msg):
        global target_x, target_y, human_found
        target_x = msg.x
        target_y = msg.y
        # print(colored(f"Yolo received pose: {target_x}, {target_y}", "red"))
        human_found = True

STATE_ONE = "START"
STATE_TWO = "EXPLORE"
STATE_THREE = "HUMAN_FOUND"
STATE_FOUR = "INFORM_RESCUE_ROBOT"
STATE_FIVE = "FINISH"

class DroneFSMBehaviour(FSMBehaviour):
    async def on_start(self):
        print(f"Drone {self.agent.id} starting at initial state {self.current_state}")

    async def on_end(self):
        print(f"Drone {self.agent.id} finished at state {self.current_state}")
        await self.agent.stop()

class StateOne(State):
    async def run(self):
        self.set_next_state(STATE_TWO)

class StateTwo(State):
    async def run(self):
        # print("Drone state EXPLORE")
        global human_found
        while not human_found:
            await asyncio.sleep(1)

        if human_found:
            self.set_next_state(STATE_THREE)
        else:
            self.set_next_state(STATE_TWO)  # Continue exploring

class StateThree(State):
    async def run(self):
        global human_found
        human_found = False
        self.set_next_state(STATE_FOUR)

class StateFour(State):
    async def run(self):
        global target_x, target_y
        msg = Message(to=self.agent.receiver_jid1)
        msg.body = str(("drone", target_x, target_y, self.agent.id))
        await self.send(msg)
        msg = Message(to=self.agent.receiver_jid2)
        msg.body = str(("drone", target_x, target_y, self.agent.id))
        await self.send(msg)
        # print(colored(f"Drone {self.agent.id} sent human location {(target_x, target_y)}", 'yellow'))
        self.set_next_state(STATE_TWO)  # After notifying, go back to exploration

class StateFive(State):
    async def run(self):
        print("Drone state FINISH (final state)")
        msg = await self.receive(timeout=5)
        if msg:
            print(f"State Five received message {msg.body}")
        self.agent.detected_human_pose_subscriber.destroy_node()
        rclpy.shutdown()

class FSMAgent(Agent):
    async def setup(self):
        global namespace
        namespace = "drone" + str(self.id)
    
        self.memory = {}
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
        self.fsm.add_transition(source=STATE_FOUR, dest=STATE_TWO)
        self.fsm.add_transition(source=STATE_TWO, dest=STATE_FIVE)
        self.add_behaviour(self.fsm)

        rclpy.init(args=None)
        self.detected_human_pose_subscriber = DetectedHumanPositionSubscriber()
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.detected_human_pose_subscriber)

        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()