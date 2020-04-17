import numpy as np 

class StanleyControl:
    def __init__(self, kp=0.5):
        self.path = None
        self.kp = kp
        self.target = 0

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
        return min_id, min_dist

    # State: [x, y, yaw, delta, v, l]
    def feedback(self, state):
        try:
            # Check Path
            if self.path is None:
                print("No path !!")
                return None, None
            
            # Extract State 
            x, y, yaw, delta, v, l = state["x"], state["y"], state["yaw"], state["delta"], state["v"], state["l"]
            
            yaw_rad = np.deg2rad(yaw)
            delta_rad = np.deg2rad(delta)

            # todo
            #############################################################################

            # all parameter name (ex:alpha) comes from the Slides
            # You need to finish the Stanley control algo

            # step by step
            # first you need to find the nearest point on the path(centered on the front wheel, previous work all on the back wheel)
            xf = x + l * np.cos(yaw_rad)
            yf = y + l * np.sin(yaw_rad)

            min_idx, _ = self._search_nearest((xf, yf))
            
            # second you need to calculate the theta_e by use the "nearest point's yaw" and "model's yaw"
            yaw_nearest = self.path[min_idx, 2]

            if yaw < -180:
                yaw = 360 + yaw
            elif yaw > 180:
                yaw = -360 + yaw

            theta_e = yaw_nearest - yaw
                    
            # third you need to calculate the v front(vf) and error(e)
            vf = v * np.sin(yaw_rad + delta_rad) + v * np.cos(yaw_rad + delta_rad)

            x_nearest = self.path[min_idx, 0]
            y_nearest = self.path[min_idx, 1]

            e = (xf - x_nearest) * np.cos(np.deg2rad(yaw_nearest + 90)) + (yf - y_nearest) * np.sin(np.deg2rad(yaw_nearest + 90))

            # now, you can calculate the delta
            delta = np.rad2deg(np.arctan2(-e * self.kp, vf)) + theta_e

            # The next_delta is Stanley Control's output
            next_delta = delta
            
            # The target is the point on the path which you find
            target = self.path[min_idx]

            # if (self.target == target): 
            #     return None, None
            # self.target = target
            
            ###############################################################################
            return next_delta, target
        except:
            return None, None
