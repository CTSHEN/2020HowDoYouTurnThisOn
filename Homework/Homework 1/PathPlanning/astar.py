import cv2
import numpy as np

class AStar():
    def __init__(self,m):
        self.map = m
        self.initialize()

    def initialize(self):
        self.queue = []
        self.parent = {}
        self.h = {} # Distance from start to node
        self.g = {} # Distance from node to goal
        self.goal_node = None

    # estimation
    def _distance(self, a, b):
        # Diagonal distance
        d = np.max([np.abs(a[0]-b[0]), np.abs(a[1]-b[1])])
        return d

    def planning(self, start=(100,200), goal=(375,520), inter=10, img=None):
        # Initialize 
        self.initialize()
        self.queue.append(start)
        self.parent[start] = None
        self.g[start] = 0
        self.h[start] = self._distance(start, goal)
        node_goal = None
        while(1):
            min_dist = 99999
            min_id = -1
            for i, node in enumerate(self.queue):
                # todo
                #####################################################
                # In a*  we need to add something in this function
                self.h[node] = self._distance(node, goal)
                f = self.g[node] + self.h[node]
                #####################################################
                if f < min_dist:
                    min_dist = f
                    min_id = i

            # pop the nearest node 
            p = self.queue.pop(min_id)

            # meet obstacle, skip
            if self.map[p[1],p[0]]<0.5:
                
                continue
            # find goal
            if self._distance(p, goal) < inter:
                self.goal_node = p
                break

            # eight direction
            pts_next1 = [(p[0]+inter,p[1]), (p[0],p[1]+inter), (p[0]-inter,p[1]), (p[0],p[1]-inter)]
            pts_next2 = [(p[0]+inter,p[1]+inter), (p[0]-inter,p[1]+inter), (p[0]-inter,p[1]-inter), (p[0]+inter,p[1]-inter)]
            pts_next = pts_next1 + pts_next2

            for pn in pts_next:
                if pn not in self.parent:
                    self.queue.append(pn)
                    self.parent[pn] = p
                    self.g[pn] = self.g[p] + inter
                    #todo
                    ##############################################
                    
                    # update the estimation
                    self.h[pn] = self._distance(p, goal)

                    ##############################################
                elif self.g[pn]>self.g[p] + inter:
                    self.parent[pn] = p
                    self.g[pn] = self.g[p] + inter
            
            if img is not None:
                cv2.circle(img,(start[0],start[1]),5,(0,0,1),3)
                cv2.circle(img,(goal[0],goal[1]),5,(0,1,0),3)
                cv2.circle(img,p,2,(0,0,1),1)
                img_ = cv2.flip(img,0)
                cv2.imshow("Homework #1 - Navigation",img_)
                k = cv2.waitKey(1)
                if k == 27:
                    break
        
        # Extract path
        path = []
        p = self.goal_node
        while(True):
            path.insert(0, p)
            if self.parent[p] == None:
                break
            p = self.parent[p]
        if path[-1] != goal:
            path.append(goal)
        return np.array(path)
