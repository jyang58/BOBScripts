# -*- coding: utf-8 -*-
"""
Created on Sun Aug 30 14:02:40 2020

@author: merli
"""
#%%

from micron_project_navigation_waypoint_maker import waypoints
from micron_project_navigation_waypoint_maker import obstacle_point_list
from micron_project_navigation_waypoint_maker import total_coordinate_grid
from ros_path_planner import *
from lamp_motion import *
from pure_pursuit import *

def main(): 
    ox = []
    oy = []
    for i in range(len(obstacle_point_list)):
        coordinate_pair = list(obstacle_point_list[i])
        ox += [coordinate_pair[0]]
        oy += [coordinate_pair[1]]
    for i in range(len(waypoints)-1):
        start_coord = waypoints[i]
        #print(start_coord)
        nav_goal = waypoints[i+1]
        #print(nav_goal)
        show_animation = True

        print(__file__ + " start!!")
    
        # start and goal position
        sx = start_coord[0]  # [m]
        print(sx)
        sy = start_coord[1]  # [m]
        print(sy)
    
        gx = nav_goal[0]  # [m]
        print(gx)
    
        gy = nav_goal[1]  # [m]
        print(gy)
    
        robot_size = 0.25  # [m]
    
        for i in range(6):
            ox.append(i)
            oy.append(0.0)
        for i in range(7):
            ox.append(6)
            oy.append(i)
        for i in range(6):
            ox.append(i)
            oy.append(6.0)
        for i in range(6):
            ox.append(0.0)
            oy.append(i)
    
    
        if show_animation:
            plt.plot(ox, oy, ".k")
            plt.plot(sx, sy, "^r")
            plt.plot(gx, gy, "^c")
            plt.grid(True)
            plt.axis("equal")
    
        rx, ry = PRM_planning(sx, sy, gx, gy, ox, oy, robot_size)
    
        assert len(rx) != 0, 'Cannot found path'
         #  target course#
        if len(rx) != 0:
            x_point_to_plot = rx
            y_point_to_plot = ry
            cx = []
            cy = []
            for i in range(len(x_point_to_plot)-1):
                x_one = x_point_to_plot[i]
                y_one = y_point_to_plot[i]
                x_two = x_point_to_plot[i+1]
                y_two = y_point_to_plot[i+1]
                m = (y_two - y_one) /(x_two - x_one)
                b = y_two - (m * x_two)
                x_coords = np.arange(x_two,x_one+.05,0.1)
                print(x_coords)
                cx += x_coords.tolist()
                for t in range(len(x_coords)):
                    x_point = x_coords[t]
                    y_point = (m*x_point)+b
                    cy += [y_point]
    
            target_speed = 1  # [m/s]
        
            T = 100.0  # max simulation time
        
            # initial state
            state = State(x=0, y=0, yaw=-3.14, v=0.0)
        
            lastIndex = len(cx) - 1
            time = 0.0
            x = [state.x]
            y = [state.y]
            yaw = [state.yaw]
            v = [state.v]
            t = [0.0]
            #MPU_Init()
            target_ind = calc_target_index(state, cx, cy)
            while T >= time and lastIndex > target_ind:
                ai = PIDControl(target_speed, state.v)
                di, target_ind = pure_pursuit_control(state, cx, cy, target_ind)
                state = update(state, ai, di)
                print(state.v)
        
                time = time + dt
        
                x.append(state.x)
                y.append(state.y)
                yaw.append(state.yaw)
                v.append(state.v)
                t.append(time)
                
                if show_animation:
                    plt.cla()
                    plt.plot(cx, cy, ".r", label="course")
                    plt.plot(x, y, "-b", label="trajectory")
                    plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
                    plt.axis("equal")
                    plt.grid(True)
                    plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
                    plt.pause(0.001)
            
            # Test
            #assert lastIndex >= target_ind, "Cannot goal"
            if lastIndex >= target_ind:
                UV_motion()
                
            if show_animation:
                plt.plot(cx, cy, ".r", label="course")
                plt.plot(x, y, "-b", label="trajectory")
                plt.legend()
                plt.xlabel("x[m]")
                plt.ylabel("y[m]")
                plt.axis("equal")
                plt.grid(True)
        
                flg, ax = plt.subplots(1)
                plt.plot(t, [iv * 3.6 for iv in v], "-r")
                plt.xlabel("Time[s]")
                plt.ylabel("Speed[km/h]")
                plt.grid(True)
                plt.show()
        
                if show_animation:
                    plt.plot(rx, ry, "-r")
                    plt.show()
        


if __name__ == '__main__':
    main()


