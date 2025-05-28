#!/usr/bin/env python3

##############################################
# Version: 1.3
# Date: 26/05/2025
# Author: Tarek Abdelmeguid 
#
# Description:
#   This ROS2 node performs path planning for an autonomous vehicle
#   navigating a simplified city modeled as a graph of nodes and edges.
#
#   The vehicle follows a sequence of goals: picking up passengers at
#   designated pickup points and then proceeding to the school.
#
#   The node subscribes to:
#     - /odom topic to get the vehicle's current position.
#     - /emergency_state topic to monitor emergency conditions.
#
#   Behavior:
#     - Wait until emergency_state is False before starting the first goal.
#     - Under normal conditions, the vehicle sequentially visits 
#       pickup1, pickup2, and finally the school.
#     - When an emergency state is detected (True), the vehicle
#       immediately reroutes to the hospital location, overriding 
#       the normal path.
#     - When the emergency clears (False), the vehicle resumes the
#       original route from its current position and current goal.
#
#   Path planning is done using the A* algorithm to find the optimal
#   path on the city graph between the current location and the active goal.
#
#   The planned path is published on the '/path_stage' topic as a
#   custom PathStage message containing:
#     - header (std_msgs/Header)
#     - poses (geometry_msgs/PoseStamped[])
#     - is_final_stage (bool)
#
##############################################

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from custom_msg.msg import PathStage
import math
import heapq


# Nodes: (x [m], y [m], node_id)
NODES = [
    (0.5, 0.6, 1), (1.5, 0.45, 2), (2.5, 0.45, 3), (3.55, 0.45, 4),
    (3.55, 1.5, 5), (3.4, 2.25, 6), (3.3, 3.5, 7), (3.4, 4.75, 8),
    (3.55, 5.5, 9), (3.55, 6.5, 10), (3.25, 7.5, 11), (2.5, 7.75, 12),
    (1.5, 7.75, 13), (0.5, 7.5, 14), (0.5, 6.5, 15), (0.5, 5.5, 16),
    (0.5, 4.75, 17), (0.5, 3.5, 18), (0.5, 2.25, 19), (0.5, 1.5, 20),
    (4.5, 0.45, 21), (5.5, 0.45, 22), (6.5, 0.45, 23), (7.25, 0.45, 24),
    (7.5, 1.5, 25), (7.25, 2.15, 26), (6.5, 2.15, 27), (5.5, 2.15, 28),
    (4.5, 2.15, 29), (2.5, 2.15, 30), (1.5, 2.25, 31), (2.5, 4.75, 32),
    (1.5, 4.75, 33)
]


# Edges: (from_id, to_id, cost)
EDGES = [
    (2, 1, 100), (3, 2, 100), (4, 3, 100), (4, 5, 100),
    (5, 6, 100), (6, 7, 100), (7, 8, 100), (8, 9, 100),
    (9, 10, 100), (10, 11, 100), (11, 12, 100), (12, 11, 100),
    (13, 12, 100), (14, 13, 100), (15, 14, 100), (16, 15, 100),
    (17, 16, 100), (18, 17, 100), (19, 18, 100), (20, 19, 100),
    (1, 20, 100), (21, 4, 100), (22, 21, 100), (23, 22, 100),
    (24, 23, 100), (25, 24, 100), (26, 25, 100), (27, 26, 100),
    (28, 27, 100), (29, 28, 100), (6, 29, 100), (30, 6, 100),
    (31, 30, 100), (19, 31, 100), (8, 32, 100), (32, 33, 100),
    (33, 17, 100), (17, 33, 100) 
]

# Special node IDs
SPECIAL_NODES = {
    'start': 22,
    'pickup1': 2,
    'pickup2': 28,
    'school': 33,
    'hospital': 11,
}

def heuristic(node_a, node_b):
    x1, y1, _ = node_a
    x2, y2, _ = node_b
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def a_star_search(start_id, goal_id):
    graph = {}
    for frm, to, cost in EDGES:
        graph.setdefault(frm, []).append((to, cost))

    start_node = next(n for n in NODES if n[2] == start_id)
    goal_node = next(n for n in NODES if n[2] == goal_id)

    open_set = []
    heapq.heappush(open_set, (0, start_id))
    came_from = {}
    g_score = {node[2]: float('inf') for node in NODES}
    g_score[start_id] = 0
    f_score = {node[2]: float('inf') for node in NODES}
    f_score[start_id] = heuristic(start_node, goal_node)

    while open_set:
        current_f, current = heapq.heappop(open_set)
        if current == goal_id:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start_id)
            path.reverse()
            return path

        for neighbor, cost in graph.get(current, []):
            tentative_g = g_score[current] + cost
            if tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(
                    next(n for n in NODES if n[2] == neighbor), goal_node)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    return []

def node_to_pose(node_id):
    node = next(n for n in NODES if n[2] == node_id)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = rclpy.clock.Clock().now().to_msg()
    pose.pose.position.x = float(node[0])
    pose.pose.position.y = float(node[1])
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 1.0
    return pose

class PathPlanningNode(Node):
    def __init__(self):
        super().__init__('path_planning_node')

        self.subscription_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.subscription_emergency = self.create_subscription(
            Bool,
            '/emergency_state',
            self.emergency_callback,
            10
        )

        # Publisher for custom PathStage message
        self.publisher_path_stage = self.create_publisher(PathStage, '/path_stage', 10)

        self.current_position = None
        self.emergency_state = True  # Start assuming emergency True to wait until False


        self.goals = [
            SPECIAL_NODES['pickup1'],
            SPECIAL_NODES['pickup2'],
            SPECIAL_NODES['school']
        ]

        self.current_goal_index = -1  # No goal selected until emergency_state == False
        self.last_published_path = []
        self.emergency_count=0
          
        self.get_logger().info("Path Planning node started")

    def odom_callback(self, msg):
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        

    def emergency_callback(self, msg):
       
        self.emergency_state = msg.data

        if self.emergency_state:
            self.plan_path()  # emergency override to hospital
        else:
            # transition from emergency → normal
            self.current_goal_index += 1
            self.plan_path()
    
    def find_next_node(self,postion):

        distances=[]
        for x , y , nid in NODES :
            dist=math.sqrt((postion[0]-x)**2 +(postion[1]-y)**2)
            distances.append((dist,nid))
        distances.sort()
        first_nearest_node=distances[0][1]
        second_nearest_node=distances[1][1]
    
        for from_id ,to_id ,cost in  EDGES:
            if (from_id == first_nearest_node and to_id == second_nearest_node) or  (from_id == second_nearest_node and to_id == first_nearest_node):
                return to_id
            
        return 0 #it couldnt found 

    
    def goal_name(self, goal_id):
    # Map goal node IDs to friendly names
        if self.emergency_state:
            return "hospital"
        else:
            if goal_id == SPECIAL_NODES['pickup1']:
                return "first pickup"
            elif goal_id == SPECIAL_NODES['pickup2']:
                return "second pickup"
            elif goal_id == SPECIAL_NODES['school']:
                return "school"
            else:
                return f"unknown ({goal_id})"


    def plan_path(self):
        if self.current_position is None:
            return
         #used to check the emergency stage
        start_node = self.find_next_node(self.current_position)
        if self.emergency_state:
            # Emergency activated: reset current_goal_index, force hospital path
            goal_id = SPECIAL_NODES['hospital']
            self.emergency_count+=1
            if self.emergency_count >1 :
                self.get_logger().info("Next goal : hospiatl")
                self.get_logger().info("stage : Final ")
                return 

        elif self.current_goal_index < len(self.goals) and self.emergency_count==0 :
            goal_id = self.goals[self.current_goal_index]

        else:
                self.get_logger().info("stage : Final ")
                self.get_logger().info("Sorry, But no more goals to advance to .")
                return
        
        stage_status=None
        if goal_id==SPECIAL_NODES['school'] or goal_id==SPECIAL_NODES['hospital']:
            stage_status='Final'
        else:
            stage_status='Waiting'

                

        path = a_star_search(start_node, goal_id)
        if not path:
            self.get_logger().error(f"No path found from {start_node} to {goal_id}") 
            return

        # Print and publish only if path changed
        if path == self.last_published_path:
            return  # Skip redundant publish/log

        self.last_published_path = path

        # Create and publish PathStage message
        path.pop(0) #remove the first element of the path to start the path from next next node 
        path_msg = PathStage()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.poses = [node_to_pose(nid) for nid in path]
        # Mark final stage if emergency or goal is school
        path_msg.is_final_stage = (goal_id == SPECIAL_NODES['hospital']) or (goal_id == SPECIAL_NODES['school']) #publish true or false based on the stage final or waiting
        self.publisher_path_stage.publish(path_msg)
        self.get_logger().info(f"Next goal :  {self.goal_name(goal_id)}")
        self.get_logger().info(f"stage: {stage_status}")
        self.get_logger().info(f"Path: {' -> '.join(str(n) for n in path)}")
    

def main(args=None):
    rclpy.init(args=args)
    node = PathPlanningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

