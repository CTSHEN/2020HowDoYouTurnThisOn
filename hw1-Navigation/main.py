import cv2
import numpy as np
from utils import *

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
isplan = False
isdone = False
iscollide = False
isSetCt = False
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


# Path Tracking Controller
#if control_type == 0:
#    from PathTracking.bicycle_pure_pursuit import PurePursuitControl
#    controller = PurePursuitControl(kp=0.1,Lfc=5)
#elif control_type == 1:
#    from PathTracking.bicycle_stanley import StanleyControl
#    controller = StanleyControl(kp=0.5)
from PathTracking.bicycle_pure_pursuit import PurePursuitControl
from PathTracking.bicycle_stanley import StanleyControl
# Path Planning Planner
#if plan_type == 0:
#    from PathPlanning.astar import AStar
#    planner = AStar(m_dilate)
#elif plan_type == 1:
#    from PathPlanning.rrt_star import RRTStar
#    planner = RRTStar(m_dilate)
from PathPlanning.cubic_spline import *
from PathPlanning.astar import AStar
from PathPlanning.rrt_star import RRTStar
##############################
# Util Function
##############################
# Mouse Click Callback
def mouse_click(event, x, y, flags, param):
    global control_type, plan_type, nav_pos, pos,  m_dilate, isplan, isdone
    if event == cv2.EVENT_LBUTTONUP:
        nav_pos_new = (x, m.shape[0]-y)
        if m_dilate[nav_pos_new[1], nav_pos_new[0]] > 0.5:
            nav_pos = nav_pos_new
            isplan = True
            isdone = True

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

##############################
# Main Function
##############################
def main():
    global nav_pos, path, init_pos, pos, isplan, isdone, isSetCt, iscollide, plan_type,control_type
    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, mouse_click)
    smooth = True ###TODO### 1.a prameter # 2. when false, error
    collide_i = 0
    target = None
    # Main Loop
    while(True):
        # Update State
        car.update()
        pos = (car.x, car.y, car.yaw)
        print("\rState: "+car.state_str(), "| Goal:", nav_pos)
        print("control_type:",control_type,"plan_type:",plan_type)  
        img_ = img.copy()

        #####################################
        # Control and Path Planning
        #####################################

        if nav_pos is not None and isplan is True:
                car.record = []
                path = None
                
                if plan_type == 0:
                    planner = AStar(m_dilate)
                    path = planner.planning(start=(int(car.x),int(car.y)), goal=nav_pos, img=img_, inter=20)
                elif plan_type ==1:
                    planner = RRTStar(m_dilate)
                    path = planner.planning((int(car.x),int(car.y)), nav_pos, 30, img_)
                if not smooth:
                    for i in range(len(path)-1):
                        cv2.line(img_, pos_int(path[i]), pos_int(path[i+1]), (1,0,0), 2)
                        path_=np.array(line_2d(path, interval=1))
                else:
                    path_ = np.array(cubic_spline_2d(path, interval=1))
                    for i in range(len(path_)-1):
                        cv2.line(img_, pos_int(path_[i]), pos_int(path_[i+1]), (1,1,0), 2)
                isplan = False
                img_ = cv2.flip(img_, 0)
                cv2.imshow(window_name ,img_)
                cv2.waitKey(1000)
        if nav_pos is not None:
                # Extract Path
            if not smooth:
                for i in range(len(path)-1):
                        cv2.line(img_, pos_int(path[i]), pos_int(path[i+1]), (1,0,0), 2)
            else:

                path_ = np.array(cubic_spline_2d(path, interval=1))
                for i in range(len(path_)-1):
                        cv2.line(img_, pos_int(path_[i]), pos_int(path_[i+1]), (1,1,0), 1)
                       
 
      
        if nav_pos is not None and isdone is True and iscollide is False and isSetCt is False :   
            state = {"x":car.x, "y":car.y, "yaw":car.yaw, "delta":car.delta,"v":car.v, "l":car.l,"dt":car.dt}
            if control_type == 0:
                controller = PurePursuitControl(kp=1,Lfc=10)
                controller.set_path(path_)
                next_delta,next_a ,target = controller.feedback(state)
                
            elif control_type == 1:
                controller = StanleyControl(kp=0.5)
                controller.set_path(path_)
                next_delta,next_a,target = controller.feedback(state)
                
            car.control(next_a, next_delta)
            if planner._distance((car.x,car.y),nav_pos)<30.0:     
                car.control(-3, next_delta)
                if car.v < 3.0:
                    car.v = 0
                    car.control(0, 0)
                    isdone = False
            isSetCt = True
        elif nav_pos is not None and isdone is True and iscollide is False and isSetCt is True:   
            state = {"x":car.x, "y":car.y, "yaw":car.yaw, "delta":car.delta,"v":car.v, "l":car.l,"dt":car.dt}
            if control_type == 0:
                controller.set_path(path_)
                next_delta,next_a ,target = controller.feedback(state)
                
            elif control_type == 1:
                controller.set_path(path_)
                next_delta,next_a,target = controller.feedback(state)
                
            car.control(next_a, next_delta)
            if planner._distance((car.x,car.y),nav_pos)<30.0:     
                car.control(-3, next_delta)
                if car.v < 3.0:
                    car.v = 0
                    car.control(0, 0)
                    isdone = False
                    isSetCt = False        
        elif nav_pos is not None and isdone is True and iscollide is True: 
            car.control(0, 0)
            car.v = -5
            collide_i += 1
            if collide_i > 100:
                iscollide = False
                collide_i = 0
                  
        # Collision Simulation
        if collision_detect(car, m):
            print("COllllide")
            #car.redo()
            iscollide = True
              
            '''
            if next_delta >= car.delta_range or next_delta <= -car.delta_range:
                car.control(0,-next_delta)
                car.v = -1.5*car.v
                print("~~~~")
            elif next_delta >0 :
                next_delta = car.delta_range 
                car.v = -2
            elif next_delta <0 :
                next_delta = -car.delta_range 
                car.v = -2
            '''
                 

            
        
        # Environment Rendering
        if nav_pos is not None:
            cv2.circle(img_,nav_pos,5,(0.5,0.5,1.0),3)
        #if target is not None:
        #    cv2.circle(img_,(int(target[0]),int(target[1])),3,(0.5,0.8,1.0),3)
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
        elif k == ord("p"):  # AStar
            plan_type = 0
        elif k == ord("o"): 
            plan_type = 1    # RRT
        elif k == ord("l"):  # PP
            control_type = 0
        elif k == ord("k"): 
            control_type = 1    # Stanley
        if k == 27:
            print()
            break

if __name__ == "__main__":
    main()
