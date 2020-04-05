import numpy as np 
import math 
import os
class StanleyControl:
    def __init__(self, kp=0.03):
        self.path = None
        self.kp = kp

    def set_path(self, path):
        self.path = path.copy()
    
    def _search_nearest(self, pos,l):
        min_dist = 99999999
        min_id = -1
        pos_f_x = l*np.cos(pos[2]) + pos[0]
        pos_f_y = l*np.sin(pos[2]) + pos[1]
        for i in range(self.path.shape[0]):
            dist = (pos_f_x - self.path[i,0])**2 + (pos_f_y - self.path[i,1])**2
            if dist < min_dist:
                min_dist = dist
                min_id = i
        return min_id, min_dist

    # State: [x, y, yaw, delta, v, l]
    def feedback(self, state):
        # Check Path
        if self.path is None:
            print("No path !!")
            return None, None
        
        # Extract State 
        x, y, yaw, delta, v, l, dt= state["x"], state["y"], state["yaw"], state["delta"], state["v"], state["l"], state["dt"]

        # todo
        #############################################################################

        # all parameter name (ex:alpha) comes from the Slides
        # You need to finish the Stanley control algo

        # step by step
        # first you need to find the nearest point on the path(centered on the front wheel, previous work all on the back wheel)
        
        target_idx, target_dist = self._search_nearest((x,y,yaw),l)
        target = self.path[target_idx, :]
        # second you need to calculate the theta_e by use the "nearest point's yaw" and "model's yaw"
        theta_e = self.path[target_idx, 2] - yaw
        # third you need to calculate the v front(vf) and error(e)
        target_v = self.path[target_idx, 3]	
        e = -(-target[0]+x)*np.sin(target[2])+ (-target[1]+y)*np.cos(target[2])


        ke = self.kp*e
        #now, you can calculate the delta
        if target_idx >= (self.path.shape[0])-1:
            next_delta = np.rad2deg( theta_e)

        else:
            next_delta =np.rad2deg(math.atan(-ke/v/np.cos(np.deg2rad(delta))) + theta_e) 
            print( target_idx,e,math.atan(-ke/v))
        self.delta =np.deg2rad( next_delta)
        # The next_delta is Stanley Control's output
        # The target is the point on the path which you find
        ###############################################################################
        ## Throtitle Control
        a = (target_v -v*np.cos(np.deg2rad(delta)))/dt
        return next_delta,a, target

if __name__ == "__main__":
    import cv2
    import path_generator
    import sys
    sys.path.append("../")
    from bicycle_model import KinematicModel

    # Path
    path = path_generator.path1()
    img_path = np.ones((600,600,3))
    for i in range(path.shape[0]-1):
        cv2.line(img_path, (int(path[i,0]), int(path[i,1])), (int(path[i+1,0]), int(path[i+1,1])), (1.0,0.5,0.5), 1)
    
    # Initialize Car
    car = KinematicModel()
    start = (50,260,0)
    car.init_state(start)
    controller = StanleyControl(kp=0.5)
    controller.set_path(path)	

    while(True):
        print("\rState: "+car.state_str()+"\t")
        
        # PID Longitude Control
        end_dist = np.hypot(path[-1,0]-car.x, path[-1,1]-car.y)
        #target_v =40 if end_dist > 265 else 0
        #next_a = 0.1*(target_v - car.v)

        # Stanley Lateral Control
        state = {"x":car.x, "y":car.y, "yaw":car.yaw, "delta":car.delta, "v":car.v, "l":car.l,"dt":car.dt}
        next_delta,next_a, target = controller.feedback(state)
        next_a = next_a   if end_dist > 80 else ( - car.v)
        car.control(next_a, next_delta)
        
        # Update State & Render
        car.update()
        img = img_path.copy()
        cv2.circle(img,(int(target[0]),int(target[1])),3,(1,0.3,0.7),2) # target points
        img = car.render(img)
        img = cv2.flip(img, 0)
        cv2.imshow("Stanley Control Test", img)
        k = cv2.waitKey(1)
        if k == ord('r'):
            car.init_state(start)
        if k == 27:
            print()
            break