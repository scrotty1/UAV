#!/usr/bin/env python3

import math
import pprint
from shapely.geometry import LineString
from shapely.geometry import Point
from shapely.ops import nearest_points

HYPOTENUSE = 1000
CURRENT_X =  600 # 0.3
CURRENT_Y = 790 # 0.3
COLLISIONS = {}
BEST_POINTS = {}

BOT_RADIUS = 1.5
GOALS = []
# obstacles are in the format (x,y,r)
OBSTACLES = []


def collision(x: float, y: float, goal_x: float, goal_y: float):
    '''
    Detects if object will collide, return point of collision
    Currently does not take into account drone hitbox
    Returns tuple with collison points or None if no collision
    '''
    path = LineString([(x,y),(goal_x,goal_y)])
    start_point = Point(x,y)
    for obstacle in OBSTACLES:
        obstacle_center = Point(obstacle[0],obstacle[1])
        obstacle_circle = obstacle_center.buffer(obstacle[2]).boundary
        inter = obstacle_circle.intersection(path)
        if type(inter) != LineString:
            p1 = Point(inter.geoms[0].x, inter.geoms[0].y)
            p2 = Point(inter.geoms[-1].x, inter.geoms[-1].y)
            if p2.distance(start_point) < p1.distance(start_point):
                return (p2.x, p2.y)
            return (p1.x,p1.y)
    return boundary_hit(x,y,goal_x,goal_y)
    

def boundary_hit(x: float, y: float, goal_x: float, goal_y: float):
    ''' returns collision data along the wall of the simulation '''
    path = LineString([(x,y),(goal_x,goal_y)])
    upper_wall = LineString([(0,800),(800,800)])
    right_wall = LineString([(800,800),(800,0)])
    left_wall = LineString([(0,0),(0,800)])
    bottom_wall = LineString([(0,0),(800,0)])

    for wall in [ upper_wall, right_wall, left_wall, bottom_wall]:
        inter = path.intersection(wall)
        if inter:
            return (inter.x, inter.y)


def detect_blocking_circle(x: float, y: float, m: float, b: float):
    ''' Checks against every obstacle to determine if there will be a collison '''
    global OBSTACLES
    for obstacle in OBSTACLES:
        # calculate the y coordinate along the line at x = center of obstacle
        closest_y = m * obstacle[0] + b
        # if that y coordinate is within the radius of the obstacle then return collision point
        if abs(closest_y - obstacle[1]) < obstacle[2]:
            return (obstacle[0], closest_y)
        '''
        A = -m
        B = 1
        C = -b
        #x_closest = ( B * (B*obstacle[0] - A*obstacle[1]) - A * C) / (A ** 2 + B ** 2)
        #y_closest = ( A * (-1*B*obstacle[0] + A*obstacle[1]) - B * C) / (A ** 2 + B ** 2)
        dist_to_center = (abs(A * obstacle[0] + B * obstacle[1] + C)) / math.sqrt(A * A + B * B)
        closest_y = m * obstacle[0] + b
        if dist_to_center < obstacle[2] + BOT_RADIUS:
            first_collision = (obstacle[0],closest_y)
        '''
    return None


def add_obstacle(x: float, y: float, r: float) -> None:
    """ Add new obstacle """
    global OBSTACLES
    OBSTACLES.insert(0, (x, y, r))


def add_goal(x: float,y: float) -> None:
    ''' Add new subgoal '''
    global GOALS
    GOALS.insert(0,(x,y))


def calculate_goals(theta):
    """
    determines the destination to use to look for a collision
    multiply by HYPOTENUSE = 1000 to account for the fact that the
    canvas is a square
    """
    return HYPOTENUSE * math.sin(math.radians(theta)), HYPOTENUSE * math.cos(math.radians(theta))


def orientation_iterator(x,y,final_goal_x, final_goal_y):
    global COLLISIONS
    """
    iterates through degrees 0-360 and finds collisions with objects
    """
    print(x,y)
    start_point = Point(x,y)
    final_goal_point = Point(final_goal_x,final_goal_y)

    for degree in range(0, 360):
        goal_x, goal_y = calculate_goals(degree)
        path = LineString([(x,y),(goal_x,goal_y)])
        np = nearest_points(path, final_goal_point)[0]
        BEST_POINTS[degree] = (np.x,np.y, final_goal_point.distance(np))

        if collision(x, y, goal_x, goal_y):
            COLLISIONS[degree] = collision(x, y, goal_x, goal_y)
        else:
            continue
        
        collision_point = Point(COLLISIONS[degree][0],COLLISIONS[degree][1])
        if start_point.distance(collision_point) < start_point.distance(np):
            BEST_POINTS[degree] = (collision_point.x,collision_point.y, final_goal_point.distance(collision_point))


if __name__ == "__main__":
    add_obstacle(1, 1, 1)
    orientation_iterator()
    pprint.pprint(COLLISIONS)
