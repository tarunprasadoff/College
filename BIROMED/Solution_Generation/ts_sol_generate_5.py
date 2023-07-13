from PIL import Image, ImageDraw
import numpy as np
from matplotlib import image, pyplot, figure, patches
import math
import random
from tqdm import tqdm
from matplotlib.animation import FuncAnimation
import pickle
import pandas as pd

tipLength = 150
linkLength = 50
noOfLinks = 6
generationDimFactor=.9
actualDimFactor=.6

img = Image.open('knee.png')

def convertImgAxArr2CoordAxArr(ar):
    return np.fliplr(ar.T)

def convertCoordAxArr2ImgAxArr(ar):
    return np.fliplr(ar).T

img_arr = np.array(img.convert('L'))
arr = convertImgAxArr2CoordAxArr(img_arr)

def checkLinkCollision(startCoord, endCoord, ar, dimFactor=generationDimFactor):
    
    ar = ar.copy()
    
    if startCoord[0] <= endCoord[0]:
        startCoord[0] = math.floor(startCoord[0])
        endCoord[0] = math.ceil(endCoord[0])
    else:
        startCoord[0] = math.ceil(startCoord[0])
        endCoord[0] = math.floor(endCoord[0])
        
    if startCoord[1] <= endCoord[1]:
        startCoord[1] = math.floor(startCoord[1])
        endCoord[1] = math.ceil(endCoord[1])
    else:
        startCoord[1] = math.ceil(startCoord[1])
        endCoord[1] = math.floor(endCoord[1])
    
    collisionCheckList = []
    collisionTolerance = 1*dimFactor # in mm
    stepInMM = .1 # from geometrical approximate
    numSteps = int( collisionTolerance / stepInMM ) + 1
    theta = math.atan2((endCoord[1]-startCoord[1]),(endCoord[0]-startCoord[0]))
    tanTheta = math.tan(theta)
    
    if -(math.pi/4) <= theta <= (math.pi/4):
        direction = 'x'
    elif (math.pi/4) <= theta <= (math.pi + (math.pi/4)):
        direction = 'y'
    elif -(math.pi + (math.pi/4)) <= theta <= -(math.pi/4):
        direction = '-y'
    else:
        direction = '-x'
    
    if '-' in direction:
        leadingStep = -1
    else:
        leadingStep = 1
    
    if 'x' in direction:
        perpendStep = tanTheta * leadingStep
        count = abs(endCoord[0]-startCoord[0])
    elif 'y' in direction:
        perpendStep = leadingStep / tanTheta
        count = abs(endCoord[1]-startCoord[1])
        
    for i in range(count):
        if 'x' in direction:
            nextCoord = ( ( startCoord[0] + ( i * leadingStep ) ) ,( startCoord[1] + round( i * perpendStep ) ) )
        elif 'y' in direction:
            nextCoord = ( ( startCoord[0] + round( i * perpendStep ) ) ,( startCoord[1] + ( i * leadingStep ) ) )
        checkPointCollision(collisionCheckList,numSteps,nextCoord)
    
#     print("Dist: ",( i * leadingStep ))
#     print("Dist: ",( i * perpendStep ))
        
    checkPointCollision(collisionCheckList,numSteps,endCoord)
        
    collisionCheckList = list(set(collisionCheckList))
    
    for coord in collisionCheckList:
        if ( coord[0] >= ar.shape[0] ) or ( coord[1] >= ar.shape[1] ):
            continue
        if ar[coord[0],coord[1]] <= 128:
            return True
        
    return False
    
def checkPointCollision(collisionCheckList, numSteps, currCoord):
    for i in range(0,(numSteps+1)):
        for j in range(0,(numSteps+1)):
            collisionCheckList.append(((currCoord[0]+i),(currCoord[1]+j)))
            collisionCheckList.append(((currCoord[0]-i),(currCoord[1]+j)))
            collisionCheckList.append(((currCoord[0]+i),(currCoord[1]-j)))
            collisionCheckList.append(((currCoord[0]-i),(currCoord[1]-j)))

def checkNodeCollision(n,isTip=True,dimFactor=generationDimFactor):
    s = [n.x, n.y]
    if isTip:
        e = [(n.x-(tipLength*math.cos(n.theta))),(n.y-(tipLength*math.sin(n.theta)))]
    else:
        e = [(n.x-(linkLength*math.cos(n.theta))),(n.y-(linkLength*math.sin(n.theta)))]
    return checkLinkCollision(s, e, arr.copy(),dimFactor=dimFactor)

class TaskSpacePlanner:
    """
    Class for Task Space planning
    """

    class Node:
        """
        Node
        """

        def __init__(self, x, y, theta):
            
            self.x = x
            self.y = y
            self.theta = theta
            self.parent = None

    def __init__(self, start, goal, fp = 10, cp = 1.5, rho = 10, alpha = 0.06, epsilon=100, max_iter=200000, fileName=None):
        
        self.start = self.Node(start[0], start[1], self.wrap_theta(start[2]))
        self.end = self.Node(goal[0], goal[1], self.wrap_theta(goal[2]))
        self.init_fp = fp
        self.fpX = fp
        self.fpY = fp
        self.cp = cp
        self.rho = rho
        self.alpha = alpha
        self.epsilon = epsilon
        self.T_goal = [self.end]
        self.max_iter = max_iter
        self.trajectoryNodes = []
        self.extNodes = []
        
        if not (fileName == None):
            self.load_state(fileName)
        
    def save_state(self, fileName):
        sf = pd.DataFrame(columns=["start","end","init_fp","fpX","fpY","cp","rho",
                                   "alpha","epsilon","T_goal","max_iter","trajectoryNodes","extNodes"])
        s = self.convertNodeToArray(self.start)
        e = self.convertNodeToArray(self.end)
        T_g = [self.convertNodeToArray(t) for t in self.T_goal]
        t_N = [self.convertNodeToArray(t) for t in self.trajectoryNodes]
        e_N = [self.convertNodeToArray(t) for t in self.extNodes]
        
        sf = sf.append({
                            "start":s,
                            "end":e,
                            "init_fp":self.init_fp,
                            "fpX":self.fpX,
                            "fpY":self.fpY,
                            "cp":self.cp,
                            "rho":self.rho,
                            "alpha":self.alpha,
                            "epsilon":self.epsilon,
                            "T_goal":T_g,
                            "max_iter":self.max_iter,
                            "trajectoryNodes":t_N,
                            "extNodes":e_N
                      }, ignore_index=True)
        
        sf.to_csv(fileName,index=False)
        
    def load_state(self, fileName):
        
        sf = pd.read_csv(fileName)
        
        self.start = self.readNodeFromArray(self.local_eval(sf.start.iloc[0]))
        self.end = self.readNodeFromArray(self.local_eval(sf.end.iloc[0]))
        self.T_goal = [self.readNodeFromArray(t) for t in self.local_eval(sf.T_goal.iloc[0])]
        self.trajectoryNodes = [self.readNodeFromArray(t) for t in self.local_eval(sf.trajectoryNodes.iloc[0])]
        self.extNodes = [self.readNodeFromArray(t) for t in self.local_eval(sf.extNodes.iloc[0])]
        self.init_fp = self.local_eval(sf.init_fp.iloc[0])
        self.fpX = self.local_eval(sf.fpX.iloc[0])
        self.fpY = self.local_eval(sf.fpY.iloc[0])
        self.cp = self.local_eval(sf.cp.iloc[0])
        self.rho = self.local_eval(sf.rho.iloc[0])
        self.alpha = self.local_eval(sf.alpha.iloc[0])
        self.epsilon = self.local_eval(sf.epsilon.iloc[0])
        self.max_iter = self.local_eval(sf.max_iter.iloc[0])

    def plan(self):        
        
        linkStartCoord = [self.end.x,self.end.y]
        linkEndX = self.end.x + ( tipLength * math.cos( self.end.theta + math.pi ) )
        linkEndY = self.end.y + ( tipLength * math.sin( self.end.theta + math.pi ) )
        linkEndCoord = [linkEndX, linkEndY]
        
        if checkLinkCollision( linkStartCoord, linkEndCoord, arr ):
            print("Start under Collision")
            return None
        
        for i in tqdm(range(self.max_iter)):
            
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.T_goal, rnd_node)
            nearest_node = self.T_goal[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node)
            
            linkStartCoord = [new_node.x,new_node.y]
            linkEndX = new_node.x + ( tipLength * math.cos( new_node.theta + math.pi ) )
            linkEndY = new_node.y + ( tipLength * math.sin( new_node.theta + math.pi ) )
            linkEndCoord = [linkEndX, linkEndY]

            if not checkLinkCollision( linkStartCoord, linkEndCoord, arr ):
                
                self.T_goal.append(new_node)
                self.fp = self.init_fp
                
#                 self.printNode(rnd_node)
                
                if self.calc_dist_to_start(self.T_goal[-1].x, self.T_goal[-1].y) <= self.epsilon:
                    
                    lastNode = self.T_goal[-1]
                    d, t = self.calc_distance_and_angle(self.start,lastNode)
                    
                    while ( d > self.rho):
                        newX = lastNode.x + ( self.rho * math.cos( t + math.pi ) )
                        newY = lastNode.y + ( self.rho * math.sin( t + math.pi ) )
                        newNode = self.Node(newX,newY,lastNode.theta)
                        newNode.parent = lastNode
                        self.T_goal.append(newNode)
                        lastNode = self.T_goal[-1]
                        d, t = self.calc_distance_and_angle(self.start,lastNode)
                    
                    oldTheta = self.T_goal[-1].theta
                    thetaDiff = self.wrap_theta(self.start.theta - oldTheta)
                    
                    if not ( abs(thetaDiff) <= self.alpha ):
                        
                        thetaDiffSign = thetaDiff / abs(thetaDiff)
                    
                        while True:

                            newTheta = oldTheta + (thetaDiffSign * self.alpha)

                            if ( abs(self.wrap_theta(self.start.theta - newTheta)) < self.alpha ):
                                break

                            self.T_goal.append(self.Node(self.T_goal[-1].x,self.T_goal[-1].y,self.wrap_theta(newTheta)))
                            self.T_goal[-1].parent = self.T_goal[-2]

                            oldTheta = newTheta
                            
                    self.start.parent = self.T_goal[-1]
                    self.T_goal.append(self.start)
                    self.generate_trajectory()
                    
                    lastNode = self.T_goal[-1]
                    
                    for i in np.arange(self.rho, (tipLength + ( linkLength * noOfLinks ) + self.rho), self.rho):
                        
                        linkEndX = self.start.x + ( i * math.cos( self.start.theta + math.pi ) )
                        linkEndY = self.start.y + ( i * math.sin( self.start.theta + math.pi ) )
                        newNode = self.Node(linkEndX,linkEndY,self.start.theta)
                        newNode.parent = lastNode
                        self.extNodes.append(newNode)
                        lastNode = newNode
                    
                    return self.T_goal
                
            else:
                
                #self.printNode(rnd_node)
                
                
                
                if self.fpX < arr.shape[0]:
                    self.fpX *= self.cp
                    self.fpX = min(self.fpX, arr.shape[0])
                    
                if self.fpY < arr.shape[1]:
                    self.fpY *= self.cp
                    self.fpY = min(self.fpY, arr.shape[1])

        return None  # cannot find path

    def steer(self, from_node, to_node):

        new_node = self.Node(from_node.x, from_node.y, self.wrap_theta(from_node.theta))
        d, theta = self.calc_distance_and_angle(new_node, to_node)
        
        new_node.x += self.rho * math.cos(theta)
        new_node.y += self.rho * math.sin(theta)
        theta_diff = self.wrap_theta(math.pi + theta - new_node.theta)
        if abs(theta_diff) == 0:
#             new_node.theta = theta
            pass
        else:
            theta_step = self.alpha * ( theta_diff / abs(theta_diff) )

            if self.alpha <= abs(theta_diff):
                new_node.theta += theta_step
            else:
                new_node.theta += theta_diff
            
        new_node.theta = self.wrap_theta(new_node.theta)

        new_node.parent = from_node

        return new_node

    def calc_dist_to_start(self, x, y):
        dx = x - self.start.x
        dy = y - self.start.y
        return math.hypot(dx, dy)

    def get_random_node(self):
        x = min( max ( 0, ( self.start.x + random.uniform(-self.fpX, self.fpX) ) ), (arr.shape[0]-1) )
        y = min( max ( 0, ( self.start.y + random.uniform(-self.fpY, self.fpY) ) ), (arr.shape[1]-1) )
        rnd = self.Node( x, y, self.wrap_theta(self.start.theta) )
        return rnd
    
    def generate_trajectory(self):
        
        self.trajectoryNodes = [self.T_goal[-1]]
        i = len(self.T_goal) - 1
        
        while i > 0:
            j = 1
            while (i-j)>=0:
                if self.isNodeEqual(self.T_goal[i].parent, self.T_goal[i-j]):
                    break
                j += 1
            self.trajectoryNodes.append(self.T_goal[i-j])
            i -= j
            
        while self.resolve_trajectory_shortcuts():
            pass
            
    def resolve_trajectory_shortcuts(self):
        
        i = 0
        while i < (len(self.trajectoryNodes)-1):
            
            currShortCut = None
            for j in range(i+2,(len(self.trajectoryNodes)-1)):
                
                if self.isNodeEqual(self.trajectoryNodes[j],self.trajectoryNodes[i].parent):
                    currShortCut = j
                    
                if not (currShortCut == None):
                    newTrajectoryNodes = self.trajectoryNodes[0:i].copy()
                    newTrajectoryNodes.extend(self.trajectoryNodes[j:].copy())
                    self.trajectoryNodes = newTrajectoryNodes
                    return True
                
            i += 1
            
        return False
    
    def readNodeFromArray(self, arr):
        return self.Node(arr[0],arr[1],arr[2])
    
    @staticmethod
    def local_eval(t):
        try:
            a = eval(t)
        except Exception as e:
            a = t
        return a
    
    @staticmethod
    def wrap_theta(theta):
        if theta >= 0:
            theta = ( theta % ( 2 * math.pi ) )
            if theta > math.pi:
                theta -= 2 * math.pi
        elif theta < 0:
            theta = - ( abs(theta) % ( 2 * math.pi ) )
            if theta <= - math.pi:
                theta += 2 * math.pi
        return theta
    
    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta

    @staticmethod
    def get_nearest_node_index(T_goal, rnd_node):
        dlist = [math.hypot((node.x - rnd_node.x),(node.y - rnd_node.y)) for node in T_goal]
        minind = dlist.index(min(dlist))

        return minind
    
    @staticmethod
    def isNodeEqual(m, n):
        return (n.x==m.x) and (n.y==m.y) and (n.theta==m.theta)
    
    @staticmethod
    def printNode(n):
        print(n.x,n.y,n.theta)
        
    @staticmethod
    def convertNodeToArray(n):
        return [n.x,n.y,n.theta]

def ts_sol_generate(startCoord, endCoord):
    print(startCoord, endCoord)
    for i in range(5):
        print(f'Attempt {i+1}')
        taskSpacePlanner = TaskSpacePlanner(start=[startCoord[0], startCoord[1], startCoord[2]],goal=[endCoord[0], endCoord[1], endCoord[2]], max_iter=25000, rho=2)
        node_list = taskSpacePlanner.plan()

        if node_list is None:
            print("Cannot find path")
        else:
            print("found path!!")
            taskSpacePlanner.save_state(f'./start_{startCoord[0]}_{startCoord[1]}_{startCoord[2]}_end_{endCoord[0]}_{endCoord[1]}_{endCoord[2]}.csv')
            # with open(f'./start_{startCoord[0]}_{startCoord[1]}_{startCoord[2]}_end_{endCoord[0]}_{endCoord[1]}_{endCoord[2]}.pkl', 'wb') as output:
            #     pickle.dump(taskSpacePlanner, output, pickle.HIGHEST_PROTOCOL)
            return None

startCoord = [950, 800, math.pi]

df = pd.read_csv('endCoords.csv')
i = 5

endCoords = [list(a) for a in list(df.iloc[(i*int(len(df)/8)):((i+1)*int(len(df)/8))].values)]

count = 0
totalCount = len(endCoords)

for endCoord in endCoords:
    ts_sol_generate(startCoord, endCoord)
    count += 1
    print(count,'/',totalCount)

print("Done")