#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Int32MultiArray
from collections import deque
import time
import heapq
import threading
import tkinter as tk
from math import cos, sin, radians

# Robot moves on intersection grid (rows+1 x cols+1)
DIRS = [(0, 1), (1, 0), (0, -1), (-1, 0)]
DIR_NAMES = ['E', 'S', 'W', 'N']
DIR_INDEX = {d: i for i, d in enumerate(DIR_NAMES)}

class FleetManager(Node):
    def __init__(self):
        super().__init__('fleet_manager')
        self.robot_ids = [1,2]
        self.rows = 4  # box grid rows
        self.cols = 3  # box grid cols

        self.start_positions = {1: ((0, 1), 'S'), 2: ((0, 1), 'S')}
        self.goal_positions = {1: (3, 3), 2: (3, 3)}

        self.obstacle_states = {rid: False for rid in self.robot_ids}
        self.route_str = {rid: "" for rid in self.robot_ids}
        self.robot_paths = {rid: [] for rid in self.robot_ids}
        self.robot_full_paths = {rid: [] for rid in self.robot_ids}  # Including return paths

        self.enable_pubs = {}
        self.route_pubs = {}

        for rid in self.robot_ids:
            self.enable_pubs[rid] = self.create_publisher(Bool, f'/robot_{rid}/line_follow_start', 10)
            self.route_pubs[rid] = self.create_publisher(String, f'/robot_{rid}/route_assign', 10)

        self.plan_collision_free_paths()
        self.publish_routes()
        self.start_all()
        self.launch_visualization()

    def get_box_approach_positions(self, box_row, box_col):
        """Get all positions around a box that a robot can approach from"""
        # Box is at (box_row, box_col) in the grid
        # Robot can approach from any adjacent intersection
        approaches = []
        
        # Top approach
        if box_row > 0:
            approaches.append((box_row, box_col))
            approaches.append((box_row, box_col + 1))
        
        # Bottom approach
        if box_row < self.rows:
            approaches.append((box_row + 1, box_col))
            approaches.append((box_row + 1, box_col + 1))
        
        # Left approach
        if box_col > 0:
            approaches.append((box_row, box_col))
            approaches.append((box_row + 1, box_col))
        
        # Right approach
        if box_col < self.cols:
            approaches.append((box_row, box_col + 1))
            approaches.append((box_row + 1, box_col + 1))
        
        # Remove duplicates while preserving order
        unique_approaches = []
        seen = set()
        for pos in approaches:
            if pos not in seen:
                unique_approaches.append(pos)
                seen.add(pos)
        
        return unique_approaches

    def determine_box_side_from_arrival(self, approach_pos, box_row, box_col, arrival_facing):
        """Determine which side of the box to access based on robot's natural arrival direction"""
        robot_row, robot_col = approach_pos
        
        # Calculate relative position of box center from robot's position
        box_center_row = box_row + 0.5
        box_center_col = box_col + 0.5
        
        # Determine left/right based on robot's arrival facing direction
        if arrival_facing == 'N':  # Robot facing North
            side = '1' if box_center_col > robot_col else '2'  # Right if box is east, Left if west
        elif arrival_facing == 'S':  # Robot facing South  
            side = '1' if box_center_col < robot_col else '2'  # Right if box is west, Left if east
        elif arrival_facing == 'E':  # Robot facing East
            side = '1' if box_center_row > robot_row else '2'  # Right if box is south, Left if north
        elif arrival_facing == 'W':  # Robot facing West
            side = '1' if box_center_row < robot_row else '2'  # Right if box is north, Left if south
        
        return side

    def plan_collision_free_paths(self):
        """Plan paths for all robots ensuring no collisions"""
        used_positions = set()
        robot_order = sorted(self.robot_ids)

        for rid in robot_order:
            start_pos, start_heading = self.start_positions[rid]
            goal_box = self.goal_positions[rid]

            # Convert goal from box to 0-indexed
            box_row, box_col = goal_box[0] - 1, goal_box[1] - 1
            approach_positions = self.get_box_approach_positions(box_row, box_col)

            best_path = None
            best_approach_pos = None
            best_cost = float('inf')

            # First attempt: try with collision avoidance
            for approach_pos in approach_positions:
                path_to_goal = self.astar_with_obstacles(start_pos, approach_pos, used_positions)
                if not path_to_goal:
                    continue
                cost = len(path_to_goal)
                if cost < best_cost:
                    best_cost = cost
                    best_path = path_to_goal
                    best_approach_pos = approach_pos

            # Retry without collision avoidance if no path found
            if not best_path:
                self.get_logger().warn(f"No collision-free path found for Robot {rid}, retrying without constraints...")
                for approach_pos in approach_positions:
                    path_to_goal = self.astar_with_obstacles(start_pos, approach_pos, set())
                    if not path_to_goal:
                        continue
                    cost = len(path_to_goal)
                    if cost < best_cost:
                        best_cost = cost
                        best_path = path_to_goal
                        best_approach_pos = approach_pos

            if not best_path:
                self.get_logger().error(f"No path found for robot {rid}")
                continue

            # Generate forward commands and determine natural arrival facing
            forward_commands, arrival_facing = self.path_to_commands(best_path, start_heading)

            # Convert direction to ABCD format using the natural arrival direction
            direction_mapping = {'N': '', 'E': '', 'S': '', 'W': ''}
            forward_commands += direction_mapping[arrival_facing]
            
            # Determine which side the goal is from robot's natural arrival position and facing
            side = self.determine_box_side_from_arrival(best_approach_pos, box_row, box_col, arrival_facing)
            forward_commands += side

            # Back away from box - robot turns around from its current facing
            opposite_dir = self.get_opposite_direction(arrival_facing)
            back_away_commands = 'T'  # simulate reverse (180 degree turn)
            current_heading = opposite_dir

            # Plan return path from approach back to start
            return_path = self.astar_with_obstacles(best_approach_pos, (2,3), set())
            if len(return_path) > 1:
                return_commands, return_final_heading = self.path_to_commands(return_path[1:], current_heading)
                back_away_commands += return_commands

                # Rotate to original heading
                rotate_to_original = self.get_turn_command(return_final_heading, start_heading)
                back_away_commands += rotate_to_original

            # Combine full command sequence
            full_commands = forward_commands + back_away_commands
            
            # Add calibration sequence at the end
            full_commands += 'TSST'

            # Store paths
            self.robot_paths[rid] = best_path
            self.robot_full_paths[rid] = best_path + return_path[1:]  # Don't reverse, it's already goal->start
            self.route_str[rid] = full_commands

            # Block path for future robots
            for pos in best_path:
                used_positions.add(pos)

            self.get_logger().info(f"Robot {rid} path: {best_path}")
            self.get_logger().info(f"Robot {rid} returns via: {return_path}")
            self.get_logger().info(f"Robot {rid} arrives facing: {arrival_facing} at {best_approach_pos}")
            self.get_logger().info(f"Robot {rid} accesses box side: {side}")
            self.get_logger().info(f"Robot {rid} commands: {full_commands}")

    def get_opposite_direction(self, direction):
        """Get opposite direction"""
        opposites = {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}
        return opposites[direction]

    def get_turn_command(self, current_dir, target_dir):
        """Get turn command to face target direction"""
        if current_dir == target_dir:
            return ""
        
        current_idx = DIR_INDEX[current_dir]
        target_idx = DIR_INDEX[target_dir]
        
        turn = (target_idx - current_idx) % 4
        
        if turn == 1:
            return 'R'
        elif turn == 2:
            return 'T'  # or 'B' for back, depending on your robot's capabilities
        elif turn == 3:
            return 'L'
        
        return ""

    def astar_with_obstacles(self, start, goal, blocked_positions):
        """A* pathfinding that avoids blocked positions"""
        def in_bounds(r, c):
            return 0 <= r <= self.rows and 0 <= c <= self.cols

        def heuristic(a, b):
            return abs(a[0]-b[0]) + abs(a[1]-b[1])

        heap = [(0 + heuristic(start, goal), 0, start, [])]
        visited = set()

        while heap:
            f, g, current, path = heapq.heappop(heap)
            if current in visited:
                continue
            visited.add(current)
            
            if current == goal:
                return path + [current]

            for i, (dr, dc) in enumerate(DIRS):
                nr, nc = current[0] + dr, current[1] + dc
                next_pos = (nr, nc)
                
                if (in_bounds(nr, nc) and 
                    next_pos not in blocked_positions and 
                    next_pos not in visited):
                    
                    new_cost = g + 1
                    heapq.heappush(heap, 
                        (new_cost + heuristic(next_pos, goal), 
                         new_cost, next_pos, path + [current]))

        return []

    def astar(self, start, goal):
        """Original A* for backward compatibility"""
        return self.astar_with_obstacles(start, goal, set())

    def path_to_commands(self, path, start_heading):
        """Convert path to movement commands"""
        if len(path) <= 1:
            return "", start_heading
            
        cmds = []
        cur_dir = DIR_INDEX[start_heading]
        
        for i in range(1, len(path)):
            r1, c1 = path[i-1]
            r2, c2 = path[i]
            
            # Find movement direction
            move_dir = None
            for idx, (dr, dc) in enumerate(DIRS):
                if (r1 + dr, c1 + dc) == (r2, c2):
                    move_dir = idx
                    break
            
            if move_dir is None:
                continue
                
            # Calculate turn needed
            turn = (move_dir - cur_dir) % 4
            
            # Add turn command first (if needed), then movement command
            if turn == 0:
                cmds.append('S')  # No turn needed, just move straight
            elif turn == 1:
                cmds.extend(['R'])  # Right turn then straight
            elif turn == 3:
                cmds.extend(['L'])  # Left turn then straight
            elif turn == 2:
                cmds.extend(['B'])  # 180 degree turn then straight
            
            cur_dir = move_dir
        
        final_heading = DIR_NAMES[cur_dir] if cur_dir is not None else start_heading
        return ''.join(cmds), final_heading

    def publish_routes(self):
        for rid in self.robot_ids:
            msg = String()
            msg.data = self.route_str[rid]
            self.route_pubs[rid].publish(msg)

    def start_all(self):
        msg = Bool()
        msg.data = True
        for pub in self.enable_pubs.values():
            pub.publish(msg)

    def launch_visualization(self):
        def draw_grid(canvas):
            # Draw path grid lines (intersections)
            for i in range(self.rows + 1):
                y = i * 100
                canvas.create_line(0, y, self.cols * 100, y, fill='black', width=2)
            for j in range(self.cols + 1):
                x = j * 100
                canvas.create_line(x, 0, x, self.rows * 100, fill='black', width=2)

            # Draw boxes (cells between lines)
            for i in range(self.rows):
                for j in range(self.cols):
                    x = (j + 0.5) * 100
                    y = (i + 0.5) * 100
                    canvas.create_rectangle(x-30, y-30, x+30, y+30, fill='lightgray', outline='black')
                    canvas.create_text(x, y, text=f"Box({i+1},{j+1})", font=('Arial', 8))

            # Draw goal boxes in different color
            for rid in self.robot_ids:
                goal_box = self.goal_positions[rid]
                box_row, box_col = goal_box[0] - 1, goal_box[1] - 1  # Convert to 0-indexed
                x = (box_col + 0.5) * 100
                y = (box_row + 0.5) * 100
                canvas.create_rectangle(x-30, y-30, x+30, y+30, fill='yellow', outline='red', width=2)
                canvas.create_text(x, y-15, text=f"Goal R{rid}", font=('Arial', 8), fill='red')

        def draw_robot_paths(canvas):
            colors = ['red', 'blue', 'green', 'purple']
            for idx, rid in enumerate(self.robot_ids):
                path = self.robot_paths[rid]
                color = colors[idx % len(colors)]
                
                # Draw path
                for i in range(1, len(path)):
                    r1, c1 = path[i-1]
                    r2, c2 = path[i]
                    x1, y1 = c1 * 100, r1 * 100
                    x2, y2 = c2 * 100, r2 * 100
                    canvas.create_line(x1, y1, x2, y2, fill=color, width=4)
                
                # Draw robot start position
                if path:
                    r, c = path[0]
                    x, y = c * 100, r * 100
                    canvas.create_oval(x-15, y-15, x+15, y+15, fill=color, outline='white', width=2)
                    canvas.create_text(x, y, text=f"R{rid}", fill='white', font=('Arial', 10, 'bold'))
                
                # Draw robot goal position
                if path:
                    r, c = path[-1]
                    x, y = c * 100, r * 100
                    canvas.create_rectangle(x-10, y-10, x+10, y+10, fill=color, outline='white', width=2)

        def run_gui():
            root = tk.Tk()
            root.title("Warehouse Robot Grid - Natural Arrival Direction")
            canvas = tk.Canvas(root, width=(self.cols+1)*100, height=(self.rows+1)*100, bg='white')
            canvas.pack()
            
            # Add legend
            legend_frame = tk.Frame(root)
            legend_frame.pack()
            
            for idx, rid in enumerate(self.robot_ids):
                color = ['red', 'blue', 'green', 'purple'][idx % 4]
                tk.Label(legend_frame, text=f"Robot {rid}: {self.route_str[rid]}", 
                        fg=color, font=('Arial', 10)).pack(side=tk.LEFT, padx=10)
            
            while True:
                canvas.delete("all")
                draw_grid(canvas)
                draw_robot_paths(canvas)
                root.update()
                time.sleep(0.1)

        threading.Thread(target=run_gui, daemon=True).start()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = FleetManager()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()