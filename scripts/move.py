#!/usr/bin/env python3
from PIL import Image
import numpy as np
from collections import deque
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class TurtleBotNavigator:
    def __init__(self, pgm_path,turtlebot_namespace,init_node=False):
        # Load and process the map
        # self.namespace=namespace
        self.pgm_image = Image.open(pgm_path)
        self.pgm_array = np.array(self.pgm_image)
        self.threshold = 127
        self.occupancy_grid = np.where(self.pgm_array > self.threshold, 0, 1)
        
        # Initialize ROS node only once (init_node is True for the first TurtleBot)
        if init_node:
            rospy.init_node(f'{turtlebot_namespace}_navigator', anonymous=True)

        # Set the correct cmd_vel topic for the specific TurtleBot
        self.vel_pub = rospy.Publisher(f'/{turtlebot_namespace}/cmd_vel', Twist, queue_size=10)

        # Subscribe to the specific TurtleBot's odometry topic
        rospy.Subscriber(f'/{turtlebot_namespace}/odom', Odometry, self.get_current_pose)


        self.current_pose = None

        self.rate = rospy.Rate(10)  # 10 Hz loop rate

  
    # === COORDINATE CONVERSION FUNCTIONS ===
    # Convert world coordinates to grid (Gazebo) coordinates
    def convert_coordinates(self, x, y):
        map_size = 383  # Assuming the map is 384x384 (0-383)
        newx = round((10 - x) * map_size / 20)
        newy = round((10 - y) * map_size / 20)
       # print(f"Converting world coordinates ({x}, {y}) to grid coordinates ({newx}, {newy})")
        return (newx, newy)

    # Convert grid (Gazebo) coordinates back to world coordinates
    def inverse_convert_coordinates(self, x, y):
        map_size = 383  # Assuming the map is 384x384 (0-383)
        newx = 10 - (x * 20 / map_size)
        newy = 10 - (y * 20 / map_size)
       # print(f"Converting grid coordinates ({x}, {y}) to world coordinates ({newx}, {newy})")
        return (newx, newy)
    
    
    def inflate_obstacles(self, grid, inflation_radius=6):  # Increase inflation radius
        rows, cols = grid.shape
        inflated_grid = grid.copy()

        for i in range(rows):
            for j in range(cols):
                if grid[i, j] == 1:  # This is an obstacle
                    # Mark cells within the inflation radius as obstacles
                    for dx in range(-inflation_radius, inflation_radius + 1):
                        for dy in range(-inflation_radius, inflation_radius + 1):
                            if 0 <= i + dx < rows and 0 <= j + dy < cols:
                                inflated_grid[i + dx, j + dy] = 1
       # print("Inflated Occupancy Grid with Coordinates:")
        # for i in range(rows):
        #     for j in range(cols):
        #         if inflated_grid[i, j] == 1:
                    # Convert grid coordinates (i, j) to world coordinates
                    world_x, world_y = self.inverse_convert_coordinates(i, j)
                   # print(f"Grid ({i}, {j}) is an obstacle. World coordinates: ({world_x}, {world_y})")

        return inflated_grid

    
    
    def is_near_obstacle(self, x, y, threshold=2):
        grid_x, grid_y = self.convert_coordinates(x, y)
        for dx in range(-threshold, threshold + 1):
            for dy in range(-threshold, threshold + 1):
                if 0 <= grid_x + dx < self.occupancy_grid.shape[0] and 0 <= grid_y + dy < self.occupancy_grid.shape[1]:
                    if self.occupancy_grid[grid_x + dx, grid_y + dy] == 1:
                        return True
        return False


    def bfs(self, grid, start, goal):
        rows, cols = grid.shape
        visited = np.zeros_like(grid, dtype=bool)
        parent = np.full((rows, cols), None)

        queue = deque([start])
        visited[start] = True
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]  # Prioritize straight movements
        diagonals = [(-1, 1), (1, 1), (-1, -1), (1, -1)]  # Add diagonal movements later if needed
        print(f"Starting BFS from {start} to {goal}")

        while queue:
            current = queue.popleft()

            if current == goal:
                print(f"Reached goal: {goal}")
                break

            for d in directions:  # First, try straight directions
                neighbor = (current[0] + d[0], current[1] + d[1])
                if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:  # Check bounds
                    if not visited[neighbor] and grid[neighbor] == 0:  # Free space and not visited
                        queue.append(neighbor)
                        visited[neighbor] = True
                        parent[neighbor] = current  # Track path

            for d in diagonals:  # Then try diagonal movements
                neighbor = (current[0] + d[0], current[1] + d[1])
                if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:  # Check bounds
                    if not visited[neighbor] and grid[neighbor] == 0:  # Free space and not visited
                        queue.append(neighbor)
                        visited[neighbor] = True
                        parent[neighbor] = current  # Track path

        # Reconstruct path from goal to start
        path = []
        step = goal
        while step is not None:
            path.append(step)
            step = parent[step]
        path.reverse()

        if path and path[0] == start:
            print("BFS path found:")
            for p in path:
                print(p)
        else:
            print("No valid path found!")
        return path if path[0] == start else []  # Ensure valid path


    # === TURTLEBOT MOVEMENT FUNCTIONS ===
    def get_yaw(self, orientation_q):
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        return yaw

    def get_current_pose(self, data):
        position = data.pose.pose.position
        orientation = data.pose.pose.orientation
        yaw = self.get_yaw(orientation)

        self.current_pose = {'x': position.x, 'y': position.y, 'yaw': yaw}
      #  print(f"Current pose: {self.current_pose}")

    def distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    # === MOVEMENT LOGIC ===
    def move_turtlebot(self, path_world):
        vel_msg = Twist()

        # Wait for odometry data
        while self.current_pose is None:
            rospy.loginfo("Waiting for odometry data...")
            self.rate.sleep()

        rospy.loginfo("Odometry data received, starting movement...")
        print(f"Following path: {path_world}")

        # Iterate over the path
        for i in range(len(path_world) - 1):
            current_point = path_world[i]
            next_point = path_world[i + 1]
            print(f"Moving from {current_point} to {next_point}")

            # Move to the next point
            while self.distance(current_point[0], current_point[1], self.current_pose['x'], self.current_pose['y']) > 0.2:  # More tolerance to prevent collisions
                if self.is_near_obstacle(self.current_pose['x'], self.current_pose['y'], threshold=5):  # Increase threshold
                    rospy.logwarn("Obstacle detected! Stopping movement.")
                    vel_msg.linear.x = 0.0
                    vel_msg.angular.z = 0.0
                    self.vel_pub.publish(vel_msg)
                    return

                # Calculate target distance and angle
                target_angle = math.atan2(next_point[1] - self.current_pose['y'], next_point[0] - self.current_pose['x'])
                angle_diff = self.normalize_angle(target_angle - self.current_pose['yaw'])

                if abs(angle_diff) > 0.1:
                    vel_msg.linear.x = 0.0  # Stop linear movement during rotation
                    vel_msg.angular.z = 0.2 * angle_diff  # Proportional rotation adjustment
                else:
                    vel_msg.angular.z = 0.0  # Stop rotation once aligned
                    vel_msg.linear.x = 0.22  # Move forward, reduced speed for safety

                self.vel_pub.publish(vel_msg)
                self.rate.sleep()

            # Stop the robot at the next point
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            self.vel_pub.publish(vel_msg)
            print(f"Reached {next_point}")
            self.rate.sleep()

        # Stop the robot after reaching the destination
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.vel_pub.publish(vel_msg)
        rospy.loginfo("Reached destination")
        
    def print_occupancy_grid(self, grid):
    # Print the grid for debugging purposes (1 = obstacle, 0 = free space)
        for row in grid:
            print(' '.join(map(str, row)))


    def follow_path_CI(self,goal_world, host_coordinate):
        print(f"CI Agent is following the path given by the BI's agent to move at the Host")
        ci_navigator.navigate(goal_world,host_coordinate)

    def follow_path_Visitor_Agent(self,start_world,goal_world):
        print(f"Visitor Agent at campus entrance gate at f{start_world} following the path given by CI agent to move to Host Location f{goal_world}")
        visitor_navigator.navigate(start_world,goal_world)

        print(f"Kudos Visitor reaches the Hosts")



    # === MAIN LOGIC ===
    def navigate(self, start_world, goal_world):
        # Convert the world coordinates to grid coordinates
        start_grid = self.convert_coordinates(*start_world)
        goal_grid = self.convert_coordinates(*goal_world)
        
        inflated_occupancy_grid = self.inflate_obstacles(self.occupancy_grid)

        # print("Inflated Occupancy Grid:")
        # self.print_occupancy_grid(inflated_occupancy_grid)

        # Run BFS to find the path in grid coordinates
        path_grid = self.bfs(inflated_occupancy_grid, start_grid, goal_grid)
        # # print(path_grid)

        # Convert the path back to world coordinates
        path_world = [self.inverse_convert_coordinates(x, y) for (x, y) in path_grid]


        # # Navigate the TurtleBot along the BFS path in world coordinates
        self.move_turtlebot(path_world)


# === MAIN EXECUTION ===
if __name__ == "__main__":
    pgm_path = './autonomous.pgm'

    # Mapping TurtleBot names to their coordinates and namespaces
    turtlebot_data = {
        "CI": {"coords": (0.0, 0.0), "namespace": "turtlebot1"},
        "B1": {"coords": (2.20, 3.25), "namespace": "turtlebot2"},
        "B2": {"coords": (-0.89, 0.99), "namespace": "turtlebot3"},
        "B3": {"coords": (5.18, -1.44), "namespace": "turtlebot4"},
        "visitor":{"coords":(0.10,-0.45),"namespace":"turtlebot5"},
    }

    # Ask user to specify which TurtleBot to move to
    goal_turtlebot = "B3"
    # input("Enter the Building name where CI Agent to move to where inside the Host is: (e.g., B1,B2,B3): ")

    if goal_turtlebot not in turtlebot_data:
        print("Invalid Building name provided!")
        exit()

    print(f"CI agent is moving from the entrance of the campus to the building {goal_turtlebot}")

    start_world = turtlebot_data["CI"]["coords"]
    goal_world = turtlebot_data[goal_turtlebot]["coords"]
    ci_namespace = turtlebot_data["CI"]["namespace"]

    # Instantiate the TurtleBotNavigator for the origin TurtleBot
    ci_navigator = TurtleBotNavigator(pgm_path,ci_namespace,init_node=True)

    # Phase 1: Navigate the origin TurtleBot to the building (B1, B2, or B3)
    ci_navigator.navigate(start_world, goal_world)

    rospy.loginfo(f"CI Agent has reached {goal_turtlebot}'s building coordinates!")

    # Phase 2: The second TurtleBot (e.g., B1) moves to a specific coordinate inside the building
    bi_namespace = turtlebot_data[goal_turtlebot]["namespace"]
    
    host_coordinate = (2.68 , -5.38)  # Host co-ordinate for that buillding where the BI's will move

    # Create a new TurtleBotNavigator instance for the second TurtleBot (e.g., B1)
    print(f"TurtleBot at {goal_turtlebot} builiding is started moving at the Host Location with coordinate {host_coordinate}")
    bi_navigator = TurtleBotNavigator(pgm_path, bi_namespace,init_node=False)
    bi_navigator.navigate(goal_world, host_coordinate)  # Bi moves to the inside building location

    rospy.loginfo(f"{goal_turtlebot} has reached to the Host Location inside the building!")

    # Phase 3: Make the CI follow the exact path that Bi's took inside the building
    rospy.loginfo(f"CI Agent is now following the path taken by the {goal_turtlebot} inside the building!")
    ci_navigator.follow_path_CI(goal_world,host_coordinate)

    
    print(f"Now the Visitor Agent will follow the path taken by the CI agent to move from the entrance fo the campus to the Host location which is at any of the building")
    visitor_namespace=turtlebot_data["visitor"]["namespace"]
    visitor_start_coordinate=turtlebot_data["visitor"]["coords"]

    visitor_navigator=TurtleBotNavigator(pgm_path,visitor_namespace,init_node=False)

    visitor_navigator.follow_path_Visitor_Agent(visitor_start_coordinate,host_coordinate)

