import asyncio
import spade
from spade.agent import Agent
from spade.behaviour import CyclicBehaviour, PeriodicBehaviour
from spade.message import Message
import math
from termcolor import colored
import datetime

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import threading

from time import sleep

class RobotPositionSubscriber(Node):
    def __init__(self, namespace):
        super().__init__(f'robot_pose_subscriber_{namespace}')
        self.namespace = namespace
        self.position = (0.0, 0.0)
        self.robot_pose_subscription = self.create_subscription(
            Odometry,
            f'/{namespace}/odom',
            self.robot_pose_callback,
            10
        )
        self.goal_pose_publisher = self.create_publisher(
            PoseStamped,
            f'/{namespace}/goal_pose',
            10
        )
        self.at_station = True
        self.station_poses = {1: (8.0, 8.0), 2: (-8.0, 8.0)}

    def robot_pose_callback(self, msg):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        # Debug: Print the current position of the robot
        # print(f"{self.namespace} pose: {self.position[0]}, {self.position[1]}")

    def calculate_distance(self, current_position, target_position):
        return math.sqrt((target_position[0] - current_position[0]) ** 2 + (target_position[1] - current_position[1]) ** 2)
    
    async def navigate_to_human(self, agent_id, drone_id, target, agent):
        print(f"Rescue robot {agent_id} is navigating to human detected by drone {drone_id}!")

        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = 'map'

        if self.at_station:
            # Create and publish goal pose
            goal_pose.pose.position.x = target[0]
            goal_pose.pose.position.y = target[1]
            goal_pose.pose.position.z = 0.0
            goal_pose.pose.orientation.x = 0.0
            goal_pose.pose.orientation.y = 0.0
            goal_pose.pose.orientation.z = 0.0
            goal_pose.pose.orientation.w = 1.0

            self.goal_pose_publisher.publish(goal_pose)
            print(f"Published goal pose for {self.namespace}")
            self.at_station = False
            agent.update_at_station_status(False)

        while not self.at_station:
            if self.calculate_distance(self.position, target) < 1.0:
                print(f"Rescue human by {self.namespace}")
                sleep(2)
                # go to station
                print(f"Go back to station {self.namespace}")
                goal_pose.pose.position.x = self.station_poses[agent_id][0]
                goal_pose.pose.position.y = self.station_poses[agent_id][1]
                self.goal_pose_publisher.publish(goal_pose)
                
            if self.calculate_distance(self.position, self.station_poses[agent_id]) < 1.0:
                print(f"Back at station {self.namespace}")
                self.at_station = True
                agent.update_at_station_status(True)
            else:
                self.at_station = False
                agent.update_at_station_status(False)

class RescueAgent(Agent):
    
    class RecvBehav(CyclicBehaviour):
        async def run(self):
            msg = await self.receive(timeout=2000)
            if msg:
                msg_str = msg.body
                msg_data = eval(msg_str)
                robot_type = msg_data[0]

                if self.agent.robot_subscriber.position[0] != 0 and self.agent.robot_subscriber.position[1] != 0:
                    if robot_type == "drone":
                        drone_id = msg_data[3]
                        self.agent.targets[drone_id] = [float(msg_data[1]), float(msg_data[2])]
                        distance = math.sqrt(
                            (self.agent.robot_subscriber.position[0] - float(msg_data[1]))**2 +
                            (self.agent.robot_subscriber.position[1] - float(msg_data[2]))**2
                        )
                        self.agent.distances[drone_id] = distance
                        print(colored(f"Robot {self.agent.id} received target from drone {drone_id}: ({msg_data[1]},{msg_data[2]}), distance to which: {distance}", 'cyan'))
                        self.agent.received_target = True

                    elif robot_type == "robot":
                        other_drone_id = msg_data[1]
                        other_robot_distance = {}
                        other_robot_distance[other_drone_id] = msg_data[2]
                        other_at_station = bool(msg_data[3])
                        if not other_at_station:
                            print(f"Info robot {self.agent.id}: other robot at station {other_at_station}")

                        if other_at_station:
                            if self.agent.distances:
                                min_distance_drone = min(self.agent.distances, key=self.agent.distances.get)
                                if self.agent.distances[min_distance_drone] < other_robot_distance.get(min_distance_drone, float('inf')):
                                    targets = self.agent.targets[min_distance_drone]
                                    await self.agent.robot_subscriber.navigate_to_human(self.agent.id, min_distance_drone, targets, self.agent)
                                    self.agent.distances = {}
                                    self.agent.targets = {}
                        else:
                            if self.agent.distances:
                                min_distance_drone = min(self.agent.distances, key=self.agent.distances.get)
                                targets = self.agent.targets[min_distance_drone]
                                await self.agent.robot_subscriber.navigate_to_human(self.agent.id, min_distance_drone, targets, self.agent)
                                self.agent.distances = {}
                                self.agent.targets = {}

                        self.agent.received_target = False
            else:
                print(f"Rescue agent {self.agent.id} did not receive any message after 10 seconds")

    class InformBehav(PeriodicBehaviour):
        async def run(self):
            # Send current status messages
            if self.agent.distances:
                messages_to_send = [
                    Message(to=self.agent.receiver_another, body=str(("robot", drone_id, distance, self.agent.robot_subscriber.at_station)))
                    for drone_id, distance in self.agent.distances.items()
                ]
                for msg in messages_to_send:
                    await self.send(msg)
            # else:
            #     for drone_id in [1, 2]:
            #         msg = Message(to=self.agent.receiver_another)
            #         msg.body = str(("robot", drone_id, 2000, self.agent.robot_subscriber.at_station))
            #         await self.send(msg)

    async def setup(self):
        self.namespace = f"robot{self.id}"
        self.distances = {}
        self.targets = {}
        self.received_target = False

        print(f"Rescue agent {self.id} started")

        recv_behav = self.RecvBehav()
        self.add_behaviour(recv_behav)

        start_at = datetime.datetime.now()
        inform_behav = self.InformBehav(period=1, start_at=start_at)
        self.add_behaviour(inform_behav)

        self.robot_subscriber = RobotPositionSubscriber(self.namespace)
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.robot_subscriber)

        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()
    
    def update_at_station_status(self, status):
        self.robot_subscriber.at_station = status
