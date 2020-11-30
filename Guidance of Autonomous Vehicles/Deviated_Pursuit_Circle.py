import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Defining the class for a follower robot

class Follower:
  def __init__(self, initCoord):
    self.coord = initCoord # Current Coordinate of the Follower Robot
    self.prevCoord = None # Previous Coordinate of the Follower Robot
    self.trajectory = [initCoord] # History of Coordinates of the Follower Robot

    self.velocity = None # Velocity of Follower Robot with Time Lag of 1
    self.theta = None # Velocity Orientation Angle of Follower Robot with Time Lag of 1
    self.velocities = [] # History of Velocities of Follower Robot at each Coordinate
    self.thetas = [] # History of Velocity Orientation Angle of Follower Robot at each Coordinate

  def step(self, velocity, theta, timeInterval):
    # Updating the values of velocity and theta
    self.velocity = velocity
    self.theta = theta
    self.velocities.append(velocity)
    self.thetas.append(theta)

    # Calculating the cartesian components of velocity
    velocityX = velocity * math.cos(theta)
    velocityY = velocity * math.sin(theta)

    # Updating Coordinates
    self.prevCoord = self.coord
    self.coord = [self.coord[0] + velocityX*timeInterval, self.coord[1] + velocityY*timeInterval] # Updating the Coordinate after a Step
    self.trajectory.append(self.coord) # Updating the History of Coordinates after a Step

# Defining a function to get Displacement Vector between Leader and Follower

def getDisplacement(coordLeader, coordFollower):
  return ((np.array(coordLeader)-np.array(coordFollower)).tolist())

# Defining a function to give Line of Sight Angle with respect to X-axis given Leader and Follower Coordinates

def getOrientationAngle(coordLeader, coordFollower):
  [deltaX, deltaY] = getDisplacement(coordLeader, coordFollower)
  return math.atan2(deltaY,deltaX)%(2*math.pi)

# Defining the function for Deviated Pursuit Velocity Computation

def dpCompute(alpha, coordLeader, coordFollower, velocityLeader, thetaLeader):
  sigma = getOrientationAngle(coordLeader, coordFollower) # Finding the Line of Sight Orientation Angle

  # Computing the magnitude and orientation of follower velocity based on given data
  thetaFollower = (sigma + alpha)%(2*math.pi)
  velocityFollower = velocityLeader * ( math.cos(thetaLeader - sigma) / math.cos(alpha) )
  return velocityFollower, thetaFollower

# Defining the initial convoy position to be along a line passing through origin

n = 5 # Number of robots in the convoy

m = 1 # Slope of the convoy line

angle = math.atan2(m,1) # Angle of the convoy line

dist = 2 # Initial distance between each convoy robot

initFollowerCoords = [([-i*dist*math.cos(angle),-i*dist*math.sin(angle)]) for i in range(1,n)] # Defining the Initial Coordinates of the Follower Robots

# Defining the trajectory of the leader of the convoy for circular case

steps = 1000 # Number of intervals in the trajectory per revolution

t = 1 # Time Interval between each step

radius = m*n*dist*1.5 # Radius of the Circle

revs = 1 # Number of revolutions

# Computing the Leader Trajectory Coordinates
circularTrajectoryCoords = [([(radius+radius*math.cos(((2*i*math.pi)/steps)+math.pi)),radius*math.sin(((2*i*math.pi)/steps)+math.pi)]) for i in range(revs*steps+1)]

# Computing the Leader Trajectory Velocity at each Coordinate
circularTrajectoryVelocities = [float(np.linalg.norm(np.array(getDisplacement(circularTrajectoryCoords[i+1],circularTrajectoryCoords[i]))/t)) for i in range(len(circularTrajectoryCoords)-1)]

# Computing the Leader Trajectory Velocity Orientation Angle at each Coordinate
circularTrajectoryVelocityOrientations = [getOrientationAngle(circularTrajectoryCoords[i+1],circularTrajectoryCoords[i]) for i in range(len(circularTrajectoryCoords)-1)]

# Initialising the Follower Robots
followers = [Follower(initFollowerCoord) for initFollowerCoord in initFollowerCoords]

# Defining the Deviation Hyperparameter
alphaDeg = -3
alpha = (math.pi/180)*alphaDeg

# Computing the trajectories for each of the Follower Robots
for step in range(revs*steps): # Iterating over each step in the trajectory
  for i in range(len(followers)): # Iterating for each Follower Robot
    # Computing the trajectory using the predefined dpCompute Function
    if (i==0):
      vTemp, thTemp = dpCompute(alpha, circularTrajectoryCoords[step], followers[i].coord, circularTrajectoryVelocities[step], circularTrajectoryVelocityOrientations[step])
    else:
      vTemp, thTemp = dpCompute(alpha, followers[i-1].prevCoord, followers[i].coord, followers[i-1].velocity, followers[i-1].theta)
    # Updating the computed trajectory using the step method in the Follower Class
    followers[i].step(vTemp, thTemp, t)

# Animating the computed trajectories for the convoy of robots

fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)

def animate(i):
  currCoordsX, currCoordsY = [], []

  currCoordsX.append(circularTrajectoryCoords[i][0])
  currCoordsY.append(circularTrajectoryCoords[i][1])

  currCoordsX.extend([follower.trajectory[i][0] for follower in followers])
  currCoordsY.extend([follower.trajectory[i][1] for follower in followers])

  ax1.clear()
  ax1.scatter(currCoordsX, currCoordsY, c ="blue")
  ax1.add_artist(plt.Circle((radius,0), radius, color='y', fill=False))

  plt.title(f"Deviated Pursuit Algorithm for a Robotic Convoy: Alpha = {alphaDeg} degrees")

  plt.xlim(-1*radius,2.25*radius)
  plt.ylim(-1.25*radius,1.25*radius)
    
ani = animation.FuncAnimation(fig, animate, interval=1, frames=len(circularTrajectoryCoords))

# Ensure the below line is uncommented to store the animation as a gif
#ani.save(f"Deviated_Pursuit_Alpha_{alphaDeg} Degrees.gif",writer='imagemagick')

plt.show()