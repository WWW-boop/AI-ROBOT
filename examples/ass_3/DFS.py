import matplotlib.pyplot as plt
import random
import time

# กำหนดทิศทาง (เหนือ, ตะวันออก, ใต้, ตะวันตก)
directions = [(-1, 0), (0, 1), (1, 0), (0, -1)]  # N, E, S, W

class RobotMazeExplorer:
    def __init__(self, maze):
        self.maze = maze
        self.n = len(maze)         # จำนวนแถว
        self.m = len(maze[0])      # จำนวนคอลัมน์
        self.start = self.random_start_position()  # ตำแหน่งเริ่มต้นแบบสุ่ม
        self.visited = [[False for _ in range(self.m)] for _ in range(self.n)]
        self.found_criminals = []  # บันทึกตำแหน่งผู้ร้ายที่เจอ
        self.found_goal = False

        # ตั้งค่ากริดสำหรับแสดงเขาวงกต
        self.fig, self.ax = plt.subplots()
        self.grid = self.maze.copy()
        self.im = self.ax.imshow(self.grid, cmap='coolwarm', interpolation='none')

    def random_start_position(self):
        """สุ่มตำแหน่งเริ่มต้นที่เป็นเส้นทาง (0)"""
        while True:
            x = random.randint(1, self.n - 2)  # ไม่รวมกำแพงด้านนอก
            y = random.randint(1, self.m - 2)
            if self.maze[x][y] == 0:  # ตรวจสอบว่าตำแหน่งเป็นเส้นทาง
                return (x, y)

    def is_valid_move(self, x, y):
        """ตรวจสอบว่าการเคลื่อนไหวถูกต้องหรือไม่ (ไม่ออกนอกเขต, ไม่ใช่กำแพง, และไม่ใช่ผู้ร้าย)"""
        if 0 <= x < self.n and 0 <= y < self.m and not self.visited[x][y] and self.maze[x][y] != 1 and self.maze[x][y] != 2:
            return True
        return False

    def update_display(self):
        """อัพเดตหน้าต่างแสดงผล"""
        self.im.set_data(self.grid)
        plt.draw()
        plt.pause(0.5)  # หน่วงเวลา

    def explore_maze(self, x, y):
        """อัลกอริธึม DFS สำหรับการสำรวจเขาวงกต"""
        if self.found_goal:
            return

        # ทำเครื่องหมายว่าตำแหน่งปัจจุบันถูกสำรวจแล้ว
        self.visited[x][y] = True
        self.grid[x][y] = 0.5  # แสดงตำแหน่งหุ่นยนต์ในสีพิเศษ
        self.update_display()

        # ตรวจสอบว่าพบผู้ร้ายหรือไม่
        if self.maze[x][y] == 2:
            print(f"เจอผู้ร้ายที่ตำแหน่ง ({x}, {y})! ขวางทางไม่สามารถไปต่อได้")
            self.found_criminals.append((x, y))
            return  # หยุดการสำรวจจากจุดนี้เพราะมีผู้ร้ายขวาง

        # สำรวจในทั้งสี่ทิศทาง
        for direction in directions:
            next_x, next_y = x + direction[0], y + direction[1]
            if self.is_valid_move(next_x, next_y):
                self.explore_maze(next_x, next_y)

        # ย้อนกลับ (Backtrack)
        if not self.found_goal:
            self.grid[x][y] = 0.8  # เปลี่ยนสีเมื่อย้อนกลับ
            self.update_display()

    def start_exploring(self):
        """เริ่มการสำรวจจากตำแหน่งเริ่มต้น"""
        print(f"เริ่มสำรวจจากตำแหน่ง: {self.start}")
        self.explore_maze(self.start[0], self.start[1])
        plt.show()  # แสดงหน้าต่างกราฟิก

# ตัวอย่างเขาวงกต: 0 -> เส้นทาง, 1 -> กำแพง, 2 -> ผู้ร้าย
maze = [
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 0, 0, 0, 0, 1, 0, 0, 0, 1],
    [1, 0, 1, 1, 0, 1, 0, 1, 0, 1],
    [1, 0, 1, 0, 0, 1, 0, 1, 0, 1],
    [1, 0, 1, 0, 1, 1, 0, 1, 0, 1],
    [1, 0, 0, 0, 1, 0, 0, 0, 0, 1],
    [1, 1, 1, 0, 1, 0, 1, 1, 1, 1],
    [1, 0, 0, 0, 0, 0, 1, 0, 2, 1],
    [1, 0, 1, 1, 1, 0, 0, 0, 0, 1],
    [1, 2, 1, 1, 1, 1, 1, 1, 1, 1]
]



# สร้าง RobotMazeExplorer ด้วยเขาวงกต
robot = RobotMazeExplorer(maze)

# เริ่มสำรวจเขาวงกตเพื่อหาผู้ร้าย
robot.start_exploring()
