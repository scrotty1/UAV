#!/usr/bin/env python3
import tkinter as tk
from lidar import *
import time

root = tk.Tk()

WIDTH=800
HEIGHT=800
D1 = 20
D2 = 30
D3 = 40

canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT)

canvas.pack()

# Placing obstacles on the canvas
x1 = 680
y1 = 680
oval1 = canvas.create_oval(x1-D1/2, y1-D1/2, x1 + D1/2, y1 + D1/2, fill="yellow")
x2 = 620
y2 = 660
oval2 = canvas.create_oval(x2-D1/2, y2-D1/2, x2 + D1/2, y2 + D1/2, fill="yellow")
x3 = 450
y3 = 500
oval3 = canvas.create_oval(x3-D2/2, y3-D2/2, x3 + D2/2, y3 + D2/2, fill="yellow")
x4 = 640
y4 = 250
oval4 = canvas.create_oval(x4-D2/2, y4-D2/2, x4 + D2/2, y4 + D2/2, fill="yellow")
x5 = 705
y5 = 320
oval5 = canvas.create_oval(x5-D3/2, y5-D3/2, x5 + D3/2, y5 + D3/2, fill="yellow")
x6 = 600
y6 = 660
oval6 = canvas.create_oval(x6-D1/2, y6-D1/2, x6 + D1/2, y6 + D1/2, fill="yellow")
x7 = 660
y7 = 500
oval7 = canvas.create_oval(x7-D3/2, y7-D3/2, x7 + D3/2, y7 + D3/2, fill="yellow")

add_obstacle(x1,y1,D1/2)
add_obstacle(x2,y2,D1/2)
add_obstacle(x3,y3,D2/2)
add_obstacle(x4,y4,D2/2)
add_obstacle(x5,y5,D3/2)
add_obstacle(x6,y6,D1/2)
add_obstacle(x7,y7,D3/2)

# Placing the bot on the canvas
bot_x = 600
bot_y = 790
bot = canvas.create_oval(bot_x, bot_y, bot_x + 3, bot_y + 3, fill="red")

# Placing the goal on the canvas
goal_x = 700
goal_y = 100
goal = canvas.create_rectangle(goal_x, goal_y, goal_x + 5, goal_y + 5, fill="green")
add_goal(goal_x,goal_y)

def lidar_scan():
    orientation_iterator(CURRENT_X,CURRENT_Y, goal_x, goal_y)
    for p in COLLISIONS:
        x_collide = COLLISIONS[p][0]
        y_collide = COLLISIONS[p][1]

        canvas.create_rectangle(x_collide,y_collide,x_collide,y_collide,fill="red")
    canvas.pack()
    

def moveto(x,y):
    global CURRENT_X
    global CURRENT_Y
    ydif = y - CURRENT_Y
    xdif = x - CURRENT_X
    magnitude = math.sqrt( ydif ** 2 + xdif ** 2)
    ydif /= magnitude
    xdif /= magnitude
    xdif *= 5
    ydif *= 5
    canvas.move(bot, xdif, ydif)
    CURRENT_X += xdif
    CURRENT_Y += ydif
    lidar_scan()
    canvas.update()


lidar_scan()
while True:
    min_distance = 100000000
    closest_x = 0
    closest_y = 0
    for i in BEST_POINTS:
        if BEST_POINTS[i][2] < min_distance:
            closest_x = BEST_POINTS[i][0]
            closest_y = BEST_POINTS[i][1]
            min_distance = BEST_POINTS[i][2]
    moveto(closest_x,closest_y)
    time.sleep(0.01)

root.mainloop()
