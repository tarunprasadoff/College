#!/usr/bin/env python
# coding: utf-8

# In[5]:


import numpy as np
import math


# In[24]:


def transmatunit(a):
    #transmatunit is a function which returns the transformation matrix from n-1 to n when feeded the DH parameters between 2 bodies 
    #a[0]=theta, a[1]=d, a[2]=a, a[3]=alpha
    tm=np.array([[math.cos(a[0]),-math.cos(a[3])*math.sin(a[0]),math.sin(a[3])*math.sin(a[0]),a[2]*math.cos(a[0])],[math.sin(a[0]),math.cos(a[3])*math.cos(a[0]),-math.sin(a[3])*math.cos(a[0]),a[2]*math.sin(a[0])],[0,math.sin(a[2]),math.cos(a[2]),a[1]],[0,0,0,1]])
    return tm


# In[53]:


#dh is a variable which stores the dh parameters between each two bodies from zeroeth to nth in a sequential order in each successive row
#transmat is a function which returns the overall transformation matrix from zeroeth to nth
def transmat(dh):
    t=np.eye(4)
    for i in dh:
        t=t.dot(transmatunit(i))   
    return t


# In[60]:


#A Puma 560 manipulator has the following parameters
#The computation is being done for the following parameter values (angles in radians, distance in cm):
theta1=1
theta2=0.5
theta3=0.25
theta4=.75
theta5=1.25
theta6=0.5
alpha1=math.pi/2
alpha2=0
alpha3=-math.pi/2
alpha4=math.pi/2
alpha5=-math.pi/2
alpha6=0
a1=0
a2=0.4318
a3=0.0203
a4=0
a5=0
a6=0
d1=0
d2=0
d3=.15
d4=0.4318
d5=0
d6=0
dh=np.array([[theta1,d1,a1,alpha1],[theta2,d2,a2,alpha2],[theta3,d3,a3,alpha3],[theta4,d4,a4,alpha4],[theta5,d5,a5,alpha5],[theta6,d6,a6,alpha6]])
transmat(dh)


# In[61]:


#The following is the transformation matrix obtained for a PUMA 560 manipulator:
#array([[ 0.10061154, -0.05496433,  0.70947666,  0.64740821],[ 0.11145638, -0.0608889 , -1.27666875, -0.2974961 ],[ 0.02444155, -0.01335248,  1.2867911 ,  0.70551305],[ 0.        ,  0.        ,  0.        ,  1.        ]])

