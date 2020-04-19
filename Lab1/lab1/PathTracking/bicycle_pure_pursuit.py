import numpy as np 
import math
PI =3.1415926535897932384626433832795
class PurePursuitControl:
    def __init__(self, kp=1, Lfc=10):
        self.path = None
        self.kp = kp
        self.Lfc = Lfc
        self.stop_speed = 0.5

    def set_path(self, path):
        self.path = path.copy()

    def _search_nearest(self, pos):
        min_dist = 99999999
        min_id = -1
        for i in range(self.path.shape[0]):
            dist = (pos[0] - self.path[i,0])**2 + (pos[1] - self.path[i,1])**2
            if dist < min_dist:
                min_dist = dist
                min_id = i
        return min_id, np.sqrt( min_dist)

    def _search_target(self, pos,Ld,ind_near):
        min_dist = 99999999
        min_id = -1
	
        for i in range(ind_near,self.path.shape[0]):
            dist = (pos[0] - self.path[i,0])**2 + (pos[1] - self.path[i,1])**2
            if (dist -Ld**2) < min_dist:
                min_dist = dist
                min_id = i
        return min_id, np.sqrt(min_dist)
    # State: [x, y, yaw, v, l]
    def feedback(self, state):
        # Check Path
        if self.path is None:
            print("No path !!")
            return None, None
        
        # Extract State 
        x, y, yaw, v, l = state["x"], state["y"], state["yaw"], state["v"], state["l"]

        # Search Front Target
        min_idx, min_dist = self._search_nearest((x,y))

        # todo
        #####################################################################
        
        # all parameter name (ex:alpha) comes from the Slides
        # You need to finish the pure pursuit control algo
        
        # step by step
        # first, you need to calculate the look ahead distance Ld by formula
	Ld = self.kp *v + self.Lfc
        # second, you need to find a point(target) on the path which distance between the path and model is as same as the Ld

        target_idx, target_dist = self._search_target((x,y),Ld,min_idx)
	target_x = self.path[target_idx,0]	
	target_y = self.path[target_idx,1]
  	target_v = self.path[target_idx,3]        
	'''	
	tangent = [cos(yaw),sin(yaw)]
	normal = [sin(yaw), -cos(yaw)]
	follow_point = closest_point + closest_tangent * lookup_distance
	'''
        ### hint: (you first need to find the nearest point and then find the point(target) backward, this will make your model won't go back)
        ### hint: (if you can not find a point(target) on the path which distance between the path and model is as same as the Ld, you need to find a similar one)
        # third, you need to calculate alpha
	alpha = np.arctan2((target_y-y),(target_x-x)) - yaw
        if alpha > 2*PI:
            alpha -= 2*PI
        print(alpha, np.arctan2(2*l*np.sin(alpha),target_dist))
	next_delta = np.rad2deg( math.atan(2*l*np.sin(alpha)/target_dist))
        # now, you can calculate the delta
	target = self.path[target_idx,:]

        # The next_delta is Pure Pursuit Control's output
        # The target is the point on the path which you find
        #####################################################################
        ## Throtitle Control
        
        if alpha !=0:
            ds = 2*alpha*Ld/np.sin(alpha)
            
        else:
            ds = target_dist
        a = (target_v**2 - v**2)/ds
        return next_delta, a, target
    def extend_path(self,cx, cy, cyaw):

        dl = 0.1
        dl_list = [dl] * (int(self.Lfc / dl) + 1)

        move_direction = math.atan2(cy[-1] - cy[-3], cx[-1] - cx[-3])
        is_back = abs(move_direction - cyaw[-1]) >= math.pi / 2.0

        for idl in dl_list:
            if is_back:
                idl *= -1
            cx = np.append(cx, cx[-1] + idl * math.cos(cyaw[-1]))
            cy = np.append(cy, cy[-1] + idl * math.sin(cyaw[-1]))
            cyaw = np.append(cyaw, cyaw[-1])

        return cx, cy, cyaw

    def set_stop_point(self,target_speed, cx, cy, cyaw):
        speed_profile = [target_speed] * len(cx)
        forward = True

        d = []

        # Set stop point
        for i in range(len(cx) - 1):
            dx = cx[i + 1] - cx[i]
            dy = cy[i + 1] - cy[i]
            d.append(math.sqrt(dx ** 2.0 + dy ** 2.0))
            iyaw = cyaw[i]
            move_direction = math.atan2(dy, dx)
            is_back = abs(move_direction - iyaw) >= math.pi / 2.0

            if dx == 0.0 and dy == 0.0:
                continue

            if is_back:
                speed_profile[i] = - target_speed
            else:
                speed_profile[i] = target_speed

            if is_back and forward:
                speed_profile[i] = 0.0
                forward = False
                #  plt.plot(cx[i], cy[i], "xb")
                #  print(iyaw, move_direction, dx, dy)
            elif not is_back and not forward:
                speed_profile[i] = 0.0
                forward = True
                #  plt.plot(cx[i], cy[i], "xb")
                #  print(iyaw, move_direction, dx, dy)
        speed_profile[0] = 0.0
        if is_back:
            speed_profile[-1] = -self.stop_speed
        else:
            speed_profile[-1] = self.stop_speed

        d.append(d[-1])

        return speed_profile, d


    def calc_speed_profile(self,cx, cy, cyaw, target_speed):

        speed_profile, d = self.set_stop_point(target_speed, cx, cy, cyaw)
        return speed_profile
    
    def closed_loop_prediction(self,cx, cy, cyaw, speed_profile, goal,state):

        
        # Extract State 
        x, y, yaw, v, l = state["x"], state["y"], state["yaw"], state["v"], state["l"]


        t = [0.0]
        a = [0.0]
        d = [0.0]
        

        return t, x, y, yaw, v, a, d#, find_goal

#---------------------------------------------------
def main2():
    import pandas as pd
    import cv2
    import path_generator
    import sys
    sys.path.append("../")
    from bicycle_model import KinematicModel
    # Path
    path = path_generator.path2()
    img_path = np.ones((600,600,3))
    for i in range(path.shape[0]-1):
        cv2.line(img_path, (int(path[i,0]), int(path[i,1])), (int(path[i+1,0]), int(path[i+1,1])), (1.0,0.5,0.5), 1)
    cx = path[:,0]
    cy = path[:,1]
    cyaw = path[:,2]
    cv = path[:,3]
    goal = [cx[-1], cy[-1]]
    target_speed = 10.0 / 3.6

    # Initialize Car
    car = KinematicModel()
    start = (50,250,0)
    car.init_state(start)
    state = {"x":car.x, "y":car.y, "yaw":car.yaw, "v":car.v, "l":car.l}
    controller = PurePursuitControl(kp=0.01, Lfc=10)
    controller.set_path(path)
    #cx, cy, cyaw = controller.extend_path(cx, cy, cyaw)
    #speed_profile = controller.calc_speed_profile(cx, cy, cyaw, target_speed)
    t, x, y, yaw, v, a, d = controller.closed_loop_prediction(
        cx, cy, cyaw, cv, goal,state)
    img = img_path.copy()
    for i in range(len(cx)):
        cv2.circle(img,(int(cx[i]),int(cy[i])),1,(1,0.3,0.7),2) # target points
    #img = cv2.flip(img, 0)
    cv2.imshow("Pure-Pursuit Control Test", img)
    k = cv2.waitKey(0)
#--------------------------------------------------
# This is for tracking the path
def main():
    import cv2
    import path_generator
    import sys
    sys.path.append("../")
    from bicycle_model import KinematicModel

    # Path
    path = path_generator.path2()
    img_path = np.ones((600,600,3))
    for i in range(path.shape[0]-1):
        cv2.line(img_path, (int(path[i,0]), int(path[i,1])), (int(path[i+1,0]), int(path[i+1,1])), (1.0,0.5,0.5), 1)

    # Initialize Car
    car = KinematicModel()
    start = (50,250,0)
    car.init_state(start)
    controller = PurePursuitControl(kp=0.01, Lfc=10)
    controller.set_path(path)

    while(True):
        print("\rState: "+car.state_str()+"\t")
        # ================= Control Algorithm ================= 
        # PID Longitude Control
        end_dist = np.hypot(path[-1,0]-car.x, path[-1,1]-car.y)
        #target_v = 40 if end_dist > 265 else 0
        #next_a = 0.1*(target_v - car.v)

        # Pure Pursuit Lateral Control
        state = {"x":car.x, "y":car.y, "yaw":car.yaw, "v":car.v, "l":car.l}
        next_delta, next_a, target = controller.feedback(state)
        next_a = next_a   if end_dist > 200 else 0.1*( - car.v)
        car.control(next_a, next_delta)
        # =====================================================
        
        # Update & Render
        car.update()
        img = img_path.copy()
        cv2.circle(img,(int(target[0]),int(target[1])),3,(1,0.3,0.7),2) # target points
        img = car.render(img)
        img = cv2.flip(img, 0)
        cv2.imshow("Pure-Pursuit Control Test", img)
        k = cv2.waitKey(1)
        if k == ord('r'):
            car.init_state(start)
        if k == 27:
            print()
            break


if __name__ == "__main__":
    main2()
