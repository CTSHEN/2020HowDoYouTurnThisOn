import numpy as np 

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
        return min_id, min_dist

    # State: [x, y, yaw, v, l]
    def feedback(self, state):
        try:
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
            kv = self.kp * v
            Ld = kv + self.Lfc

            # second, you need to find a point(target) on the path which distance between the path and model is as same as the Ld
            ### hint: (you first need to find the nearest point and then find the point(target) backward, this will make your model won't go back)
            ### hint: (if you can not find a point(target) on the path which distance between the path and model is as same as the Ld, you need to find a similar one)
            for i in range(min_idx, self.path.shape[0]):
                distance = np.sqrt((self.path[i,0] - self.path[min_idx, 0]) ** 2 + (self.path[i, 1] - self.path[min_idx, 1]) ** 2)
                if distance >= Ld:
                    target_idx = i
                    break
            
            # third, you need to calculate alpha
            yaw = np.deg2rad(yaw)
            alpha = np.arctan2(y - self.path[target_idx, 1], x - self.path[target_idx, 0]) - yaw

            # now, you can calculate the delta
            delta = -np.arctan2(2 * l * np.sin(alpha), Ld)

            # The next_delta is Pure Pursuit Control's output
            next_delta = np.rad2deg(delta)

            # The target is the point on the path which you find
            target = self.path[target_idx]

            #####################################################################
            return next_delta, target
        except:
            return None, None
