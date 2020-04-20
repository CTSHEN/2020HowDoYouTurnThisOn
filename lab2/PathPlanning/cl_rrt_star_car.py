import cv2
import numpy as np
from utils import *
import math
import random
import Lab1.lab1.PathTracking.bicycle_pure_pursuit
rho =10
STEP_SIZE = 0.1
class RRTStar():
    def __init__(self,m):
        self.map = m

    def _distance(self, n1, n2):
        d = np.array(n1[:3]) - np.array(n2[:3])
        return np.sqrt(d[0]**2 + d[1]**2 + d[2]**2)#np.hypot(d[0], d[1])

    def _random_node(self, goal, shape,state,r_):
        r = np.random.choice(3,1,p=[0.3,0.3,0.4])

        if r==1:
            return (float(goal[0]), float(goal[1]), float(goal[2]))
    
        else:
            rx = float(np.random.randint(int(shape[1])))
            ry = float(np.random.randint(int(shape[0])))
            rth = float(random.uniform(-math.pi, math.pi))   
            #r = r_ + np.random.rand(1)*300
            #th = start[2] + np.random.rand(1)*3.14
            #rx_ = float(start[0] + r * np.cos(th))
            #ry_ = float(start[1] + r * np.sin(th))
            
            return (rx, ry,rth)

    def _nearest_node(self, samp_node):
        min_dist = 99999
        min_node = None
        for n in self.ntree:
            dist = self._distance(n, samp_node)
            if dist < min_dist:
                min_dist = dist
                min_node = n
        return min_node

    def _check_collision(self, n1, n2):
        n1_ = pos_int(n1)
        n2_ = pos_int(n2)
        line = Bresenham(n1_[0], n2_[0], n1_[1], n2_[1])
        for pts in line:
            if self.map[int(pts[1]),int(pts[0])]<0.5:
                return True
        return False

    def _steer(self, from_node, to_node, extend_len,goal,start):
        vect = np.array(to_node[:2]) - np.array(from_node[:2])
        v_len = np.hypot(vect[0], vect[1])
        v_theta = np.arctan2(vect[1], vect[0])
        vect1 = np.array(goal[:2]) - np.array(from_node[:2])
        theta2goal = np.arctan2(vect1[1], vect1[0])
        px, py, pyaw, mode, clen = reeds_shepp_path_planning.reeds_shepp_path_planning(
            from_node[0], from_node[1], from_node[2],
            to_node[0], to_node[1], to_node[2], 5, STEP_SIZE)
        

        # at least extend_len
        if extend_len > v_len:
            extend_len = v_len
        new_node = (from_node[0]+extend_len*np.cos(v_theta), from_node[1]+extend_len*np.sin(v_theta),v_theta)
        # 
        ####################################################################################################################################################
        # this "if-statement" is not complete, you need complete this "if-statement"
        # you need to check the path is legal or illegal, you can use the function "self._check_collision"
        # illegal
        if new_node[1]<0 or new_node[1]>=self.map.shape[0] or new_node[0]<0 or new_node[0]>=self.map.shape[1] or self._check_collision(from_node,to_node): 
        ####################################################################################################################################################
            return False, None
            '''
            elif from_node is start and np.fabs(np.cos(start[2])*vect[0]+np.sin(start[2])*vect[1])/v_len>0.5:
            
            p1 = (new_node[0]- from_node[0])*np.cos(from_node[2]) + (new_node[1]- from_node[1])*np.sin(from_node[2])
            p2 = (new_node[1]- from_node[1])*np.sin(from_node[2]) - (new_node[0]- from_node[0])*np.sin(from_node[2])
            dc = (p1)**2+ (math.fabs(p2) -rho)**2
            dis = np.sqrt(dc -rho**2) + rho*(np.arctan2(p1,rho-math.fabs(p2))- np.arccos(rho/np.sqrt(dc)))
            if dis > 0.1:
                return False, None
            else: 
                return new_node, self._distance(new_node, from_node)
            '''
        # legal
        else:
            p1 = (new_node[0]- from_node[0])*np.cos(from_node[2]) + (new_node[1]- from_node[1])*np.sin(from_node[2])
            p2 = (new_node[1]- from_node[1])*np.sin(from_node[2]) - (new_node[0]- from_node[0])*np.sin(from_node[2])
            dc = (p1)**2+ (math.fabs(p2) -rho)**2
            dis = np.sqrt(dc -rho**2) + rho*(np.arctan2(p1,rho-math.fabs(p2))- np.arccos(rho/np.sqrt(dc)))     
            return new_node,  self._distance(new_node, from_node)
    
    def _near_node(self, node, radius):
        nlist = []
        for n in self.ntree:
            if n == node or self._check_collision(n,node):
                continue
            if self._distance(n, node) <= radius:
                nlist.append(n)
        return nlist

    def planning(self, start, goal, extend_lens, img=None):
        self.ntree = {}
        self.ntree[start] = None
        self.cost = {}
        self.cost[start] = 0
        goal_node = None
        for it in range(20000):
            #print("\r", it, len(self.ntree))
            samp_node = self._random_node(goal, self.map.shape,start,30)
            near_node = self._nearest_node(samp_node)

            cv2.waitKey(10)
            new_node, cost,px,py,pyaw = self._steer(near_node, samp_node, extend_lens,goal,start)
            if new_node is not False:
                # todo
                ###################################################################
                # after creat a new node in a tree, we need to maintain something
                self.ntree[new_node] = near_node 
                self.cost[new_node] = cost + self.cost[near_node]
                ###################################################################
            else:
                continue
            if self._distance(near_node, goal) < extend_lens:
                goal_node = near_node
                break
        
            # Re-Parent
            nlist = self._near_node(new_node, 200)
            for n in nlist:
                '''
            	p1 = (new_node[0]- n[0])*np.cos(n[2])
            	p2 = (new_node[1]- n[1])*np.sin(n[2])
                p1 = (new_node[0]- n[0])*np.cos(n[2]) + (new_node[1]- n[1])*np.sin(n[2])
                p2 = (new_node[1]- n[1])*np.sin(n[2]) - (new_node[0]- n[0])*np.sin(n[2])
            	dc = (p1)**2+ (math.fabs(p2) -rho)**2
            	dis = np.sqrt(dc -rho**2) + rho*(np.arctan2(p1,rho-math.fabs(p2))- np.arccos(rho/np.sqrt(dc))) 
                if n is start and dis > 0.2 :
                    continue 
                '''      
                cost = self.cost[n] + self._distance(n, new_node)
                if cost < self.cost[new_node]:
                    # todo
                    ###################################################################
                    self.ntree[new_node] = n 
                    self.cost[new_node] = cost 
                    ###################################################################

            # Re-Wire
            for n in nlist:
                '''
            	p1 = (n[0]- new_node[0])*np.cos(new_node[2])
            	p2 = (n[1]- new_node[1])*np.sin(new_node[2])
                p1 = (n[0]- new_node[0])*np.cos(new_node[2]) + (n[1]- new_node[1])*np.sin(new_node[2])
                p2 = (n[1]- new_node[1])*np.sin(new_node[2]) - (n[0]- new_node[0])*np.sin(new_node[2])
            	dc = (p1)**2+ (math.fabs(p2) -rho)**2
            	dis = np.sqrt(dc -rho**2) + rho*(np.arctan2(p1,rho-math.fabs(p2))- np.arccos(rho/np.sqrt(dc)))  
                if n is start and dis > 0.2 :
                    continue
                ''' 
                cost = self.cost[new_node] +self._distance(n, new_node)
                if cost < self.cost[n]:
                    # todo
                    ###################################################################
                    # update the near node's distance
                    self.ntree[new_node] = n 
                    self.cost[new_node] = cost 
                   ###################################################################

            # Draw
            if img is not None:
                for n in self.ntree:
                    if self.ntree[n] is None:
                        continue
                    node = self.ntree[n]
                    cv2.line(img, (int(n[0]), int(n[1])), (int(node[0]), int(node[1])), (1,0,0), 1)
                # Near Node
                img_ = img.copy()
                cv2.circle(img_,pos_int(new_node),5,(0,0.5,1),3)
                for n in nlist:
                    cv2.circle(img_,pos_int(n),3,(0,0.7,1),2)
                # Draw Image
                img_ = cv2.flip(img_,0)
                cv2.imshow("RRT* Test",img_)
                k = cv2.waitKey(1)
                if k == 27:
                    break
        
        # Extract Path
        path = []
        n = goal_node
        while(True):
            if n is None:
                break
            path.insert(0,n)
            node = self.ntree[n]
            n = self.ntree[n] 
        path.append(goal)
        return path

def pos_int(p):
    return (int(p[0]), int(p[1]))

smooth = True
if __name__ == "__main__":
    # Config
    img = cv2.flip(cv2.imread("../Maps/map.png"),0)
    img[img>128] = 255
    img[img<=128] = 0
    m = np.asarray(img)
    m = cv2.cvtColor(m, cv2.COLOR_RGB2GRAY)
    m = m.astype(float) / 255.
    m = 1-cv2.dilate(1-m, np.ones((20,20)))
    img = img.astype(float)/255.

    start=(100,200,-0.5)
    goal=(380,520,0)
    cv2.circle(img,(start[0],start[1]),5,(0,0,1),3)
    cv2.circle(img,(goal[0],goal[1]),5,(0,1,0),3)

    rrt = RRTStar(m)
    path = rrt.planning(start, goal, 40, img)
    
    # Extract Path
    if not smooth:
        for i in range(len(path)-1):
            cv2.line(img, pos_int(path[i]), pos_int(path[i+1]), (0.5,0.5,1), 3)
    else:
        from cubic_spline import *
        path = np.array(cubic_spline_2d(path, interval=4))
        for i in range(len(path)-1):
            cv2.line(img, pos_int(path[i]), pos_int(path[i+1]), (0.5,0.5,1), 3)

    img_ = cv2.flip(img,0)
    cv2.imshow("RRT* Test",img_)
    k = cv2.waitKey(0)
