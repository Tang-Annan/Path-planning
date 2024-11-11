import matplotlib.pyplot as plt
import numpy as np

def draw_horizontal_line(start_x, start_y, length, o_x, o_y, o_dict):
    for i in range(start_x, start_x + length):
        for j in range(start_y, start_y + 2):
            o_x.append(i)
            o_y.append(j)
            o_dict[(i, j)] = True


def draw_vertical_line(start_x, start_y, length, o_x, o_y, o_dict):
    for i in range(start_x, start_x + 2):
        for j in range(start_y, start_y + length):
            o_x.append(i)
            o_y.append(j)
            o_dict[(i, j)] = True

obs_dict = {}
for i in range(51):
    for j in range(51):
        obs_dict[(i, j)] = False
ox,oy = [],[]

horizontal_line_data = [(0,0,50),(0,48,50),(35, 5, 10), (40, 10, 5), (15, 15, 10), (10, 20, 10), (45, 20, 5), (20, 25, 5), (10, 30, 10), (15, 35, 5), (25, 35, 10), (45, 35, 5), (10, 40, 10), (30, 40, 5), (10, 45, 5), (40, 45, 5)]
vertical_line_data = [(0,0,50),(48,0,50),(10, 10, 10), (10, 30, 10), (10, 45, 5), (15, 20, 10), (20, 5, 10), (20, 40, 5), (30, 10, 20), (30, 40, 10), (35, 5, 25), (30, 40, 10), (40, 10, 35), (45, 25, 15)]

# horizontal_line_data = [(-10,-10,60),(-10,48,60),(35, 5, 10), (40, 10, 5), (15, 15, 10), (10, 20, 10), (45, 20, 5), (20, 25, 5), (10, 30, 10), (15, 35, 5), (25, 35, 10), (45, 35, 5), (10, 40, 10), (30, 40, 5), (10, 45, 5), (40, 45, 5),(0,0,15),(20,-5,10),(-5,25,10)]
# vertical_line_data = [(-10,-10,60),(48,-10,60),(10, 10, 10), (10, 30, 10), (10, 45, 5), (15, 20, 10), (20, 5, 10), (20, 40, 5), (30, 10, 20), (30, 40, 10), (35, 5, 25), (30, 40, 10), (40, 10, 35), (45, 25, 15),(0,0,15),(30,-5,10),(-5,25,15)]

for data in horizontal_line_data:
        draw_horizontal_line(data[0], data[1], data[2], ox, oy,obs_dict)

for data in vertical_line_data:
        draw_vertical_line(data[0], data[1], data[2], ox, oy,obs_dict)

sx,sy = 5, 5
# sx,sy = -5, -5
gx,gy = 38, 40


if __name__ == '__main__':
    plt.plot(ox,oy,"sk")
    plt.axis("equal")
    plt.plot(sx,sy,".g",markersize=10)
    plt.plot(gx,gy,"r*",markersize=10)
    plt.grid(True)
    plt.show()