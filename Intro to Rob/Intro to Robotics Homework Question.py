#!/usr/bin/env python
# coding: utf-8

# In[2]:


import numpy
print("Please enter the elements of the transformation matrix in a sequential row-wise manner")

N = numpy.zeros([4,4])

for i in range(4):
	for j in range(4):
		N[i,j] = input() #Receiving the transformation matrix elements

q3 = 777 -int(N[2][3]) #Equation for d3

c2 = pow(int(N[0][3]),2) + pow(int(N[1][3]),2) - 375*375 - 425*425 #Equation for Cos Theta

s2 = pow(1-c2,1/2) #Equation for Sin Theta

q2 = numpy.arctan2(s2,c2)

r = 375*c2 + 425 #Equation for r in terms of theta

s = 375*s2 #Equation for s in terms of theta

q1 = numpy.arctan2(-r*int(N[1][3])-s*int(N[0][3]), int(N[0][3])*r -s*int(N[1][3]))

q4 = numpy.arctan2(-int(N[0][1]),int(N[0][0])) - q1 - q2

print("The joint variables of the manipulator are as follows:", q1*180/numpy.pi, " " ,q2*180/numpy.pi , " ", q3, " " ,q4*180/numpy.pi, " ")


# In[ ]:




