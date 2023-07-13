from PIL import Image, ImageDraw
import numpy as np
from matplotlib import image, pyplot, figure, patches
import math
import random
from tqdm import tqdm
from time import sleep
from matplotlib.animation import FuncAnimation
import pickle
import sys
import scipy.interpolate as si
import pandas as pd
import os

# currIndex = int(os.path.basename(__file__).split('.py')[0].split('_')[-1])

tipLength = 150
linkLength = 50
noOfLinks = 6
generationDimFactor=0.9
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

def plotLine(startCoord=None, endCoord=None, currCoords = None, withMarker=True, color='g'):
    if currCoords == None:
        currCoords = [startCoord, endCoord]
    if withMarker:
        pyplot.plot([c[0] for c in currCoords],[(arr.shape[1]-c[1]) for c in currCoords],linestyle='-',color=color,marker='o')
    else:
        pyplot.plot([c[0] for c in currCoords],[(arr.shape[1]-c[1]) for c in currCoords],linestyle='-',color=color)

def plotNode(n,withMarker=True,isTip=True,shouldCheck=True,dimFactor=generationDimFactor):
    s = [n.x, n.y]
    if isTip:
        e = [(n.x-(tipLength*math.cos(n.theta))),(n.y-(tipLength*math.sin(n.theta)))]
    else:
        e = [(n.x-(linkLength*math.cos(n.theta))),(n.y-(linkLength*math.sin(n.theta)))]
    color = 'g'
    if checkLinkCollision(s, e, arr.copy(),dimFactor=dimFactor) and shouldCheck:
        color='r'
    plotLine(s,e,withMarker=withMarker,color=color)

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

            if not checkLinkCollision( linkStartCoord, linkEndCoord, arr, dimFactor=actualDimFactor):
                
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
        if random.random()>0.25:
            x = min( max ( 0, ( self.start.x + random.uniform(-self.fpX, self.fpX) ) ), (arr.shape[0]-1) )
            y = min( max ( 0, ( self.start.y + random.uniform(-self.fpY, self.fpY) ) ), (arr.shape[1]-1) )
        else:
            x = min( max ( 0, ( self.start.x + random.uniform(-self.init_fp, self.init_fp) ) ), (arr.shape[0]-1) )
            y = min( max ( 0, ( self.start.y + random.uniform(-self.init_fp, self.init_fp) ) ), (arr.shape[1]-1) )
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

fileNames = [o for o in os.listdir('Data') if '.csv' in o]

def generateTSPlot(fileName, taskSpacePlanner):
    f = pyplot.figure()
    f.set_figheight(f.get_figheight()*3)
    f.set_figwidth(f.get_figwidth()*5)
    ar = 255*np.ones([(arr.shape[0]+400),arr.shape[1]])
    ar[0:arr.shape[0],:] = arr.copy()
    pyplot.imshow(convertCoordAxArr2ImgAxArr(ar))
    del ar
    [plotNode(n,withMarker=False,dimFactor=actualDimFactor) for n in taskSpacePlanner.trajectoryNodes]
    if not (len(taskSpacePlanner.trajectoryNodes) == 0):
        plotLine(currCoords=[[t.x,t.y] for t in taskSpacePlanner.trajectoryNodes],color='k',withMarker=False)
    pyplot.title("Main Branch Nodes")
    plotName = f"Plots/{fileName.split('.csv')[0]}.png"
    pyplot.savefig(plotName)
    return plotName

def getSmooth(coords, d, n):

    def bspline(cv, n=100, degree=3):
        """ Calculate n samples on a bspline

            cv :      Array ov control vertices
            n  :      Number of samples to return
            degree:   Curve degree
        """
        cv = np.asarray(cv)
        count = cv.shape[0]

        # Prevent degree from exceeding count-1, otherwise splev will crash
        degree = np.clip(degree,1,count-1)

        # Calculate knot vector
        kv = np.array([0]*degree + list(range(count-degree+1)) + [count-degree]*degree,dtype='int')

        # Calculate query range
        u = np.linspace(0,(count-degree),n)

        # Calculate result
        return np.array(si.splev(u, (kv,cv.T,degree))).T
    
    cv = np.array(coords)

    #pyplot.plot(cv[:,0],cv[:,1], 'o-', label='Control Points')

    p = bspline(cv,n=n,degree=d)
    x,y = p.T
    
    coordsSmooth = [[x[i],y[i]] for i in range(len(x))]
    
    return coordsSmooth

def getCheckPoint(x,y,checkList,checkPointIndex):
    dists = []
    for i in range(len(checkList)):
        if (i < checkPointIndex):
            dists.append(math.inf)
        else:
            n = checkList[i]
            dx = n.x - x
            dy = n.y - y
            
            dists.append(math.hypot(dx, dy))
    checkPointIndex = dists.index(min(dists)) + 1
    return checkPointIndex

def checkNext(nodeLists, index, isFull):
    
    nextThetaDiffs = [taskSpacePlanner.wrap_theta(nodeLists[-1][i].theta-nodeLists[-1][i-1].theta) for i in range(1,len(nodeLists[-1]))]
    if index == 0:
        return getFinal(nextThetaDiffs, nodeLists, isFull)
    nl = []
    n = taskSpacePlanner.trajectoryNodes[index-1]
    n.theta = nodeLists[-1][0].theta
    currTheta = n.theta
    nl.append(n)
    l = tipLength
    collisions = []
    collisions.append(checkNodeCollision(n,isTip=(tipLength==l), dimFactor=actualDimFactor))
    for nextThetaDiff in nextThetaDiffs:
        x = n.x - l * math.cos(n.theta)
        y = n.y - l * math.sin(n.theta)
        currTheta = taskSpacePlanner.wrap_theta( currTheta + nextThetaDiff )
        n = taskSpacePlanner.Node(x,y,currTheta)
        nl.append(n)
        l = linkLength
        collisions.append(checkNodeCollision(n,isTip=(tipLength==l), dimFactor=actualDimFactor))
        
    if True in collisions:
        if len(nodeLists) == 1:
            print("No Node List without collision in the next instant")
            if isFull and (not isFull=="Done"):
                return None
            else:
                raise Exception('No Solution')
        else:
            return checkNext(nodeLists[:-1], index, isFull)
    else:
        return getFinal(nextThetaDiffs, nodeLists, isFull)

def getFinal(nextThetaDiffs, nodeLists, isFull):
    if isFull == "Done":
        return nextThetaDiffs, nodeLists, isFull
    elif isFull == True:
        return nextThetaDiffs, nodeLists, nodeLists[-1][0]
    else:
        return nextThetaDiffs, nodeLists

def plotNodeList(i, currNodeList=None, shouldPlot=False, nodeList=None, dimFactor=actualDimFactor, checkList = None):
    
    global f
    pyplot.clf()
    
    if nodeList == None:
        nodeList = nodeSuperList
    
    if shouldPlot:
        f = pyplot.figure()
        f.set_figheight(f.get_figheight()*3)
        f.set_figwidth(f.get_figwidth()*5)
    
    ar = 255*np.ones([(arr.shape[0]+400),arr.shape[1]])
    ar[0:arr.shape[0],:] = arr.copy()
        
    pyplot.imshow(convertCoordAxArr2ImgAxArr(ar))
    
    if not (checkList == None):
        pyplot.plot([t.x for t in checkList],[(arr.shape[1]-t.y) for t in checkList],'k')
       
    if ( currNodeList == None ):
        currNodeList = nodeList[i]
    
    isTip = True
    for n in currNodeList:
        plotNode(n, isTip=isTip, dimFactor=dimFactor)
        isTip = False
        
    # if shouldPlot:
    #     pyplot.show()

def computeCurrThetaStepCorrection(prevNode, prevLength, currLength, maxThetaDeviation, checkList, checkPointIndex, pastThetaDiff = None, considerMid=False, firstDiff=None):
    
    currX = prevNode.x - prevLength*math.cos(prevNode.theta)
    currY = prevNode.y - prevLength*math.sin(prevNode.theta)

    distToTheta = {}
    
    prevTheta = prevNode.theta
    
#     if not (pastThetaDiff == None):
#         pastCurrTheta = taskSpacePlanner.wrap_theta( prevTheta + pastThetaDiff )
#     else:
#         pastCurrTheta = taskSpacePlanner.wrap_theta(prevTheta)
    
    tUpperLimit = prevTheta + maxThetaDeviation
    tLowerLimit = prevTheta - maxThetaDeviation

    thetaIntervals = 1000
    tStep = abs( tUpperLimit - tLowerLimit ) / thetaIntervals
    
    for t in list(np.arange(tLowerLimit,tUpperLimit,tStep)):
        
        x = currX - currLength * math.cos(t)
        y = currY - currLength * math.sin(t)
        
        x2 = currX - currLength * math.cos(t) * .5
        y2 = currY - currLength * math.sin(t) * .5
        
        dists = []
        
        for i in range(len(checkList)):
            
            if (i < checkPointIndex):
                dists.append(math.inf)
            else:
                n = checkList[i]
                
                dx = n.x - x
                dy = n.y - y
                
                dx2 = n.x - x2
                dy2 = n.y - y2
                
                if considerMid:
                    dists.append(math.hypot(dx, dy) + (.5 * math.hypot(dx2, dy2)))
                else:
                    dists.append(math.hypot(dx, dy))
            
        distToTheta[min(dists)] = t
    
    minDistTheta = None
    while ( minDistTheta == None ):
        if (firstDiff == None):
            if len(distToTheta)==0:
                print("No Thetas without Collision")
                raise Exception('No Solution')
            minDist = min(list(distToTheta.keys()))
            minDistTheta = distToTheta[minDist]
            currTheta = taskSpacePlanner.wrap_theta(minDistTheta)
            if checkNodeCollision(taskSpacePlanner.Node(currX,currY,currTheta) ,isTip=False, dimFactor=actualDimFactor):
                minDistTheta = None
                distToTheta.pop(minDist)
        else:
            if len(distToTheta)==0:
                print("No First Thetas without Collision in next Instant")
                raise Exception('No Solution')
            minDist = min(list(distToTheta.keys()))
            minDistTheta = distToTheta[minDist]
            currTheta = taskSpacePlanner.wrap_theta(minDistTheta)
            if checkNodeCollision(taskSpacePlanner.Node(currX,currY,currTheta) ,isTip=False, dimFactor=actualDimFactor) and checkNodeCollision(taskSpacePlanner.Node((currX+firstDiff[0]),(currY+firstDiff[1]),currTheta) ,isTip=False, dimFactor=actualDimFactor):
                minDistTheta = None
                distToTheta.pop(minDist)
    
    thetaDiff = currTheta-prevTheta
    
    x = currX - currLength * math.cos(minDistTheta)
    y = currY - currLength * math.sin(minDistTheta)
    checkPointIndexNew = getCheckPoint(x,y,checkList,checkPointIndex)
            
#     return currX, currY, currTheta, thetaDiff, checkPointIndexNew, pastCurrTheta
    return currX, currY, currTheta, thetaDiff, checkPointIndexNew

def solveConfigStepCorrection(noOfLinks=4,currThetaDiffs=[], nodeIndex=-1, checkListType='Original', considerMid=False, var=None):
    
    nextThetaDiffs = [None for _ in range(noOfLinks-1)]
    isFirst = False
    if len(currThetaDiffs) < (noOfLinks-1):
        isFirst = True
        currThetaDiffs = nextThetaDiffs.copy()
    
    currentThetas = []
    currentThetaPrevs = []
    currentNodes = []

    n = taskSpacePlanner.trajectoryNodes[nodeIndex]
    currentNodes.append(n)
    
    checkList = getCheckList(checkListType,var=var)
    
    x = n.x - tipLength * math.cos(n.theta)
    y = n.y - tipLength * math.sin(n.theta)
    checkPointIndex = 0
    checkPointIndex = getCheckPoint(x,y,checkList,checkPointIndex)
    
    if isFirst:
        firstDiff = [(taskSpacePlanner.trajectoryNodes[nodeIndex-1].x-n.x),(taskSpacePlanner.trajectoryNodes[nodeIndex-1].y-n.y)]
    else:
        firstDiff = None

    for i in range(0,noOfLinks-1):
        
        if i == 0:
            l = tipLength
        else:
            l = linkLength
        
        xi, yi, thetai, nextThetaDiffs[i], checkPointIndex = computeCurrThetaStepCorrection(n,l,linkLength,(math.pi/3),checkList,checkPointIndex,currThetaDiffs[i], considerMid=considerMid, firstDiff=firstDiff)
        n = taskSpacePlanner.Node(xi, yi, thetai)
        currentNodes.append(n)
        currentThetas.append(n.theta)
        
    if isFirst:
        return nextThetaDiffs, [currentNodes]
    
    currentThetaPrevs = [taskSpacePlanner.wrap_theta(taskSpacePlanner.trajectoryNodes[nodeIndex].theta+currThetaDiffs[0])]
    for i in range(1,len(currThetaDiffs)):
        currentThetaPrevs.append(taskSpacePlanner.wrap_theta(currentThetaPrevs[-1]+currThetaDiffs[i]))
    
    thetaMoves = [taskSpacePlanner.wrap_theta(currentThetas[i] - currentThetaPrevs[i]) for i in range(len(currentThetas))]
    absThetaMoves = [abs(t) for t in thetaMoves]
    maxThetaMove = max(absThetaMoves)
    noSteps = int(maxThetaMove/taskSpacePlanner.alpha)
    stepMove = [ ( t / ( noSteps ) ) for t in thetaMoves]
    
    thetaLists = []
    
    for i in range(0,noSteps):
        thetaLists.append([])
        for j in range(len(currentThetas)):
            thetaLists[-1].append( taskSpacePlanner.wrap_theta( currentThetaPrevs[j] + ( i * stepMove[j] ) ) )
            
    thetaLists.append(currentThetas)
    
    nodeLists = []
    
    for thetaList in thetaLists:
        nodeLists.append([])
        n = taskSpacePlanner.trajectoryNodes[nodeIndex]
        x = n.x - tipLength * math.cos(n.theta)
        y = n.y - tipLength * math.sin(n.theta)
        nodeLists[-1].append(n)
        for t in thetaList:
            n = taskSpacePlanner.Node(x, y, t)
            x = n.x - linkLength * math.cos(n.theta)
            y = n.y - linkLength * math.sin(n.theta)
            nodeLists[-1].append(n)
            
            if checkNodeCollision(n,isTip=False,dimFactor=actualDimFactor):
                if thetaList == thetaLists[0]:
                    print("No Solution in this direction")
                    plotNodeList(None,currNodeList=nodeLists[-1],shouldPlot=True, dimFactor=actualDimFactor, checkList=checkList)
                    raise Exception('No Solution')
                #print("Early Exit due to Collision")
                
                #nextThetaDiffs = [taskSpacePlanner.wrap_theta(nodeLists[:-1][-1][i].theta-nodeLists[:-1][-1][i-1].theta) for i in range(1,len(nodeLists[:-1][-1]))]
                #return nextThetaDiffs, nodeLists[:-1]
                return checkNext(nodeLists[:-1], nodeIndex, False)
            
        #nextThetaDiffs = [taskSpacePlanner.wrap_theta(nodeLists[-1][i].theta-nodeLists[-1][i-1].theta) for i in range(1,len(nodeLists[-1]))]
    
    #return nextThetaDiffs, nodeLists
    return checkNext(nodeLists, nodeIndex, False)

def smoothTrajectory(trajectory):
    trajectoryCoordsSmooth = getSmooth([[t.x,t.y] for t in trajectory], 3, int(len(trajectory)/3))
    return [TaskSpacePlanner.Node(t[0],t[1],math.pi) for t in trajectoryCoordsSmooth]

def getCheckList(checkListType, var=None):
    if 'Original' in checkListType:
        checkList = [t for t in reversed(taskSpacePlanner.trajectoryNodes)]
        if 'Smooth' in checkListType:
            checkList = smoothTrajectory(checkList)
        checkList.extend(taskSpacePlanner.extNodes)
    elif 'Median' in checkListType:
        checkList = [taskSpacePlanner.Node((t.x+(var*tipLength*math.cos(t.theta+math.pi))),(t.y+(var*tipLength*math.sin(t.theta+math.pi))),0) for t in reversed(taskSpacePlanner.trajectoryNodes)]
        if 'Smooth' in checkListType:
            checkList = smoothTrajectory(checkList)
        checkList.extend([taskSpacePlanner.Node((n.x+(var*tipLength*math.cos(taskSpacePlanner.start.theta+math.pi))),(n.y+(var*tipLength*math.sin(taskSpacePlanner.start.theta+math.pi))),n.theta) for n in taskSpacePlanner.extNodes])
    return checkList

def convertNodeListsToArray(nodeLists):
    return [[TaskSpacePlanner.convertNodeToArray(n) for n in ne] for ne in nodeLists]

def convertArrayToNodeList(array):
    return [[TaskSpacePlanner(start=[950, 800, math.pi],goal=[265, 580, (math.pi*1.3)], max_iter=5000).readNodeFromArray(n) for n in ne] for ne in array]

dataDfName = 'res.csv'
if dataDfName in os.listdir('./'):
    dataDf = pd.read_csv(dataDfName)
else:
    dataDf = pd.DataFrame(columns=["fileName","plotName","nodeSuperList","checkListType","var","simName","status"])
dataDf.to_csv(dataDfName,index=False)

def plotSimulation(nodeSuperList, fileName, checkListType, var):

    print('Length of NodeSuperList',len(nodeSuperList))
    f = pyplot.figure()
    f.set_figheight(f.get_figheight()*3)
    f.set_figwidth(f.get_figwidth()*5)

    anim = FuncAnimation(f, plotNodeList, frames = len(nodeSuperList), interval = 40)
    simName = f"Simulations/{fileName.split('.csv')[0]}_{checkListType}_{'None' if var==None else var}.gif"
    anim.save(simName,writer='Pillow')
    return simName

checkListTypes = [('Original',None),('OriginalSmooth',None),('Median',0.25),('MedianSmooth',0.25),('Median',0.5),('MedianSmooth',0.5),('Median',0.75),('MedianSmooth',0.75)]

fileNameCounter = 0
fileNameLength = len(fileNames)*len(checkListTypes)

for fileName in tqdm(fileNames):
    
    for (checkListType,var) in checkListTypes:
        
        if (len(dataDf[dataDf["fileName"]==fileName][dataDf["var"]==var][dataDf["checkListType"]==checkListType]) == 1) or (len(dataDf[dataDf["fileName"]==fileName][dataDf["var"].isnull()][dataDf["checkListType"]==checkListType]) == 1):
            continue
            
        taskSpacePlanner = TaskSpacePlanner(start=[950, 800, math.pi],goal=[265, 580, (math.pi*1.3)], max_iter=5000)
        taskSpacePlanner.load_state('Data/'+fileName)
        plotName = generateTSPlot(fileName,taskSpacePlanner)
        status = 'Fail'
        
        try:        
            nd = []
            nodeSuperList = []
            for i in tqdm(range(len(taskSpacePlanner.trajectoryNodes))):
                nd, nl = solveConfigStepCorrection(noOfLinks=noOfLinks,currThetaDiffs=nd,nodeIndex=(len(taskSpacePlanner.trajectoryNodes)-1-i),checkListType=checkListType,var=var)
                nodeSuperList.extend(nl)
            status = 'Pass'
        except Exception as e:
            print(e)
            
        nodeList = convertNodeListsToArray(nodeSuperList)
        simName = None
        if len(nodeSuperList)>0:
            simName = plotSimulation(nodeSuperList, fileName, checkListType, var)
        dataDf = dataDf.append({"fileName":fileName,"plotName":plotName,"nodeSuperList":nodeList,"checkListType":checkListType,"var":var,"simName":simName,"status":status},ignore_index=True)
        dataDf.to_csv(dataDfName,index=False)

        fileNameCounter += 1
        print(f'{fileNameCounter}/{fileNameLength}')