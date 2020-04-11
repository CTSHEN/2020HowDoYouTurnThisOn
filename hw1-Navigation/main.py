import cv2
import numpy as np
from utils import *
from PathPlanning.cubic_spline import *

##############################
# Preset
##############################
# Algorithm Setting
# 0: Pure_pursuit / 1: Stanley
control_type = 0
# 0: Astar / 1: RRT Star
plan_type = 0

# Global Information
nav_pos = None
init_pos = (100,200,0)
pos = init_pos
window_name = "Homework #1 - Navigation"

# Read Image
img = cv2.flip(cv2.imread("Maps/map.png"),0)
img[img>128] = 255
img[img<=128] = 0
m = np.asarray(img)
m = cv2.cvtColor(m, cv2.COLOR_RGB2GRAY)
m = m.astype(float) / 255.
m_dilate = 1-cv2.dilate(1-m, np.ones((40,40))) # Configuration-Space
img = img.astype(float)/255.

# Simulation Model
from bicycle_model import KinematicModel
car = KinematicModel(l=20, d=5, wu=5, wv=2, car_w=14, car_f=25, car_r=5)
car.init_state(init_pos)

'''
# Path Tracking Controller
if control_type == 0:
    from PathTracking.bicycle_pure_pursuit import PurePursuitControl
    controller = PurePursuitControl(kp=0.7,Lfc=10)
elif control_type == 1:
    from PathTracking.bicycle_stanley import StanleyControl
    controller = StanleyControl(kp=0.5)

# Path Planning Planner
if plan_type == 0:
    from PathPlanning.astar import AStar
    planner = AStar(m_dilate)
elif plan_type == 1:
    from PathPlanning.rrt_star import RRTStar
    planner = RRTStar(m_dilate)
from PathPlanning.cubic_spline import *
'''

##############################
# Util Function
##############################
# Mouse Click Callback
def mouse_click(event, x, y, flags, param):
    global control_type, plan_type, nav_pos, pos,  m_dilate
    if event == cv2.EVENT_LBUTTONUP:
        nav_pos_new = (x, m.shape[0]-y)
        if m_dilate[nav_pos_new[1], nav_pos_new[0]] > 0.5:
            nav_pos = nav_pos_new

def collision_detect(car, m):
    p1,p2,p3,p4 = car.car_box
    l1 = Bresenham(p1[0], p2[0], p1[1], p2[1])
    l2 = Bresenham(p2[0], p3[0], p2[1], p3[1])
    l3 = Bresenham(p3[0], p4[0], p3[1], p4[1])
    l4 = Bresenham(p4[0], p1[0], p4[1], p1[1])
    check = l1+l2+l3+l4
    collision = False
    for pts in check:
        if m[int(pts[1]),int(pts[0])]<0.5:
            collision = True
            break
    return collision

#////////////////////////////////////////////////////////////////////////
def Path_Planning_Planner(plan_type=1):
    global planner, path, img_path
    smooth = True # False
    start = (np.int(pos[0]), np.int(pos[1]))
    goal = (np.int(nav_pos[0]),np.int(nav_pos[1]))
    if plan_type == 0:
        from PathPlanning.Astra import AStar
        planner = AStar(m_dilate)
        path = planner.planning(start=start, goal=goal, inter=20, img=img)
        print("\rAStar path =")
        print(path)
        cv2.circle(img, (start[0], start[1]), 5, (0, 0, 1), 3)
        cv2.circle(img, (goal[0], goal[1]), 5, (0, 1, 0), 3)
        # Extract Path
        if not smooth:
            for i in range(len(path) - 1):
                cv2.line(img, path[i], path[i + 1], (1, 0, 0), 1)#2
        else:
            # from cubic_spline import *
            path = np.array(cubic_spline_2d(path, interval=1))
            for i in range(len(path) - 1):
                cv2.line(img, pos_int(path[i]), pos_int(path[i + 1]), (1, 0, 0), 1)
        img_path = cv2.flip(img, 0)
        # cv2.imshow("A* Test", img_)
        # k = cv2.waitKey(0)
        print("\rAStar path 1=")
        print(path)
    elif plan_type == 1:
        from PathPlanning.rrt_star import RRTStar
        planner = RRTStar(m_dilate)
        path = planner.planning(start, goal, 30, img)

        cv2.circle(img, (start[0], start[1]), 5, (0, 0, 1), 3)
        cv2.circle(img, (goal[0], goal[1]), 5, (0, 1, 0), 3)
        # Extract Path
        if not smooth:
            for i in range(len(path) - 1):
                cv2.line(img, pos_int(path[i]), pos_int(path[i + 1]), (0.5, 0.5, 1), 3)
        else:
            # from cubic_spline import *
            path = np.array(cubic_spline_2d(path, interval=4))
            for i in range(len(path) - 1):
                cv2.line(img, pos_int(path[i]), pos_int(path[i + 1]), (0.5, 0.5, 1), 3)
        img_path = cv2.flip(img, 0)
        # cv2.imshow("RRT* Test", img_)
        # k = cv2.waitKey(0)

#////////////////////////////////////////////////////////////////////////
#////////////////////////////////////////////////////////////////////////
def Path_Tracking_Controller(image=None, control_type=0):
    global controller
    path_Controller = []
    if control_type == 0:
        from PathTracking.bicycle_pure_pursuit import PurePursuitControl
        controller = PurePursuitControl(kp=0.7, Lfc=10)
        print("\rpath 1=")
        print(path)
        #for i in range(path.shape[0] - 1):
        #    cv2.line(img_path, (int(path[i, 0]), int(path[i, 1])), (int(path[i + 1, 0]), int(path[i + 1, 1])),(1.0, 0.5, 0.5), 1)
        #    path_Controller.append([(int(path[i, 0]), int(path[i, 1]), int(path[i + 1, 0]), int(path[i + 1, 1]))])
        #print("\rpath_Controller=")
        #print(path_Controller)
        controller.set_path(path)
        #while (True):
        print("\rState: " + car.state_str(), end="\t")
        # ================= Control Algorithm =================
        # PID Longitude Control
        #Goal: (403, 407)
        #(100, 200), (120, 200), (140, 200), (160, 200), (180, 200), (200, 220),
        #[100, 200, 100, 200], [100, 200, 101, 200], [101, 200, 102, 200], [102, 200, 103, 200],
        #[402, 406, 402, 407]
        end_dist = np.hypot(path[-1, 0] - car.x, path[-1, 1] - car.y)
        target_v = 40 if end_dist > 265 else 0
        next_a = 0.1 * (target_v - car.v)

        # Pure Pursuit Lateral Control
        state = {"x": car.x, "y": car.y, "yaw": car.yaw, "v": car.v, "l": car.l}
        next_delta, target = controller.feedback(state)
        car.control(next_a, next_delta)
        # =====================================================
        print("\rState: " + car.state_str(), "| Goal:", nav_pos, end="\t")

        """
        # Update & Render
        car.update()
        img_ = image.copy()
        cv2.circle(image, (int(target[0]), int(target[1])), 3, (1, 0.3, 0.7), 2)  # target points
        img_ = car.render(img_)
        img_ = cv2.flip(img_, 0)
        cv2.imshow("Pure-Pursuit Control Test", img_)
        k = cv2.waitKey(1)
        if k == ord('r'):
            car.init_state(pos)
        if k == 27:
            print()
            break #[346, 380, 347, 380]
        """
    elif control_type == 1:
        from PathTracking.bicycle_stanley import StanleyControl
        controller = StanleyControl(kp=0.5)
        controller.set_path(path)
        while (True):
            print("\rState: " + car.state_str(), end="\t")

            # PID Longitude Control
            end_dist = np.hypot(path[-1, 0] - car.x, path[-1, 1] - car.y)
            target_v = 40 if end_dist > 265 else 0
            next_a = 0.1 * (target_v - car.v)

            # Stanley Lateral Control
            state = {"x": car.x, "y": car.y, "yaw": car.yaw, "delta": car.delta, "v": car.v, "l": car.l}
            next_delta, target = controller.feedback(state)
            car.control(next_a, next_delta)

            # Update State & Render
            car.update()
            img_ = image.copy()
            cv2.circle(img_, (int(target[0]), int(target[1])), 3, (1, 0.3, 0.7), 2)  # target points
            img_ = car.render(img_)
            img_ = cv2.flip(img_, 0)
            cv2.imshow("Stanley Control Test", img_)
            k = cv2.waitKey(1)
            if k == ord('r'):
                car.init_state(pos)
            if k == 27:
                print()
                break

#////////////////////////////////////////////////////////////////////////

##############################
# Main Function
##############################
def main():
    global nav_pos, path, init_pos, pos, flag
    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, mouse_click)
    flag = 0
    # Main Loop
    while(True):
        # Update State
        car.update()
        pos = (car.x, car.y, car.yaw)
        print("\rState: "+car.state_str(), "| Goal:", nav_pos, end="\t")
        img_ = img.copy()
        #////////////////////////////////////////////////////////////////////////
        #mouse_click(event=0, x=100, y=200, flags=0, param=0)
        if nav_pos is not None and pos != nav_pos:
            if flag == 0:
                # Control and Path Planning
                # Path Planning Planner
                # 0: Astar / 1: RRT Star
                Path_Planning_Planner(plan_type = 0)
                flag = 1
            #if nav_pos is not None and pos != nav_pos:
            # Path Tracking Controller
            # 0: Pure_pursuit / 1: Stanley
            # print("\r#################111111")
            Path_Tracking_Controller(image=img, control_type=0)
        if pos == nav_pos:
            flag = 0
        #////////////////////////////////////////////////////////////////////////

        # Collision Simulation
        if collision_detect(car, m):
            car.redo()
            car.v = -0.5*car.v
        
        # Environment Rendering
        if nav_pos is not None:
            cv2.circle(img_,nav_pos,5,(0.5,0.5,1.0),3)
        img_ = car.render(img_)
        img_ = cv2.flip(img_, 0)
        cv2.imshow(window_name ,img_)

        # Keyboard 
        k = cv2.waitKey(1)
        if k == ord("a"):
            car.delta += 5
        elif k == ord("d"):
            car.delta -= 5
        elif k == ord("w"):
            car.v += 4
        elif k == ord("s"):
            car.v -= 4
        elif k == ord("r"):
            car.init_state(init_pos)
            nav_pos = None
            path = None
            print("Reset!!")
        if k == 27:
            print()
            break

if __name__ == "__main__":
    main()