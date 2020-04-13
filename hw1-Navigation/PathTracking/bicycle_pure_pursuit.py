import numpy as np 
import math
PI =3.1415926535897932384626433832795
class PurePursuitControl:
    def __init__(self, kp=1, Lfc=10):
        self.path = None
        self.kp = kp
        self.Lfc = Lfc

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
    start = (50,270,0)
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
