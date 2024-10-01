import random

# Direction constants
UP, RIGHT, DOWN, LEFT = 0, 1, 2, 3
DIRECTIONS = [(0, -1), (1, 0), (0, 1), (-1, 0)]  # (dx, dy) for moving in directions

# Initial setup for the maze (assuming a 6x6 grid)
maze_size = 6
maze = [['#' for _ in range(maze_size)] for _ in range(maze_size)]  # Unknown cells as walls ('#')

# Initialize the robot's starting position in the center
robot_pos = [maze_size // 2, maze_size // 2]
maze[robot_pos[1]][robot_pos[0]] = 'S'  # Mark the start position

visited = [[False for _ in range(maze_size)] for _ in range(maze_size)]  # Keep track of visited cells

def detect_wall(direction):
    """
    Simulate wall detection in the given direction.
    This function should use real sensor input from the robot.
    """
    # Replace with real sensor data
    return random.choice([True, False])  # Randomize for now (True = wall, False = no wall)

def move_forward():
    """
    Move the robot forward by 1 block.
    Update robot's position accordingly.
    """
    global robot_pos, current_direction
    dx, dy = DIRECTIONS[current_direction]
    robot_pos[0] += dx
    robot_pos[1] += dy

def turn_left():
    """
    Turn the robot 90 degrees to the left.
    """
    global current_direction
    current_direction = (current_direction - 1) % 4

def turn_right():
    """
    Turn the robot 90 degrees to the right.
    """
    global current_direction
    current_direction = (current_direction + 1) % 4

def dfs(x, y, direction):
    """
    DFS to explore all paths in the maze.
    """
    visited[y][x] = True  # Mark the current cell as visited

    for i in range(4):  # Try all directions (up, right, down, left)
        next_direction = (direction + i) % 4
        dx, dy = DIRECTIONS[next_direction]
        nx, ny = x + dx, y + dy

        # If within bounds and not yet visited
        if 0 <= nx < maze_size and 0 <= ny < maze_size and not visited[ny][nx]:
            # Check if there's a wall in the direction
            if not detect_wall(next_direction):
                # Move the robot in the direction
                move_forward()

                # Update the maze map
                maze[ny][nx] = ' '

                # Recursively explore the next position
                dfs(nx, ny, next_direction)

                # Backtrack: Move the robot back and update position
                turn_left()
                turn_left()
                move_forward()
                turn_left()
                turn_left()

# Initial direction facing up (0 = UP)
current_direction = UP

# Start DFS exploration from the robot's current position
dfs(robot_pos[0], robot_pos[1], current_direction)

# Print out the explored maze
for row in maze:
    print(' '.join(row))
