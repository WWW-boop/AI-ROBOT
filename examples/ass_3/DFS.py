# การนำเข้าโมดูลที่จำเป็น
from robomaster import robot
import time


ep_robot = robot.Robot()
ep_robot.initialize(conn_type='sta')

maze_size = 6  # ขนาดของเขาวงกต 6x6
cell_size = 120  # ขนาดของแต่ละช่อง 120 cm
visited = [[False] * maze_size for _ in range(maze_size)]  # บันทึกช่องที่สำรวจแล้ว

# ฟังก์ชันการเคลื่อนที่
def move_to_cell(x, y):
    # ฟังก์ชันนี้จะเคลื่อนที่หุ่นยนต์ไปยังตำแหน่ง (x, y) ในแผนที่
    ep_robot.chassis.drive_speed(x=0.3, y=0, z=0)  # เคลื่อนที่ไปข้างหน้า
    time.sleep(1)  # ปรับให้เหมาะสมกับระยะทางที่ต้องการเคลื่อนที่
    ep_robot.chassis.drive_speed(x=0, y=0, z=0)  # หยุด

# ฟังก์ชัน DFS สำหรับการสำรวจ
def dfs(x, y):
    visited[x][y] = True  # ทำเครื่องหมายช่องที่สำรวจแล้ว
    move_to_cell(x, y)  # เคลื่อนที่ไปยังช่องนี้

    # ลองสำรวจช่องที่อยู่รอบๆ
    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:  # ขึ้น, ลง, ซ้าย, ขวา
        new_x, new_y = x + dx, y + dy
        if 0 <= new_x < maze_size and 0 <= new_y < maze_size and not visited[new_x][new_y]:
            dfs(new_x, new_y)  # สำรวจช่องถัดไป


try:
    dfs(0, 0)
except KeyboardInterrupt:
    ep_robot.close()  
