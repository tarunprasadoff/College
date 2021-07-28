import numpy as np
from math import exp as exp

pyx = np.array([.604,.8184,.9546,.9915,.9497,.857])

def pxfunc(x):
    return ( exp(-((x-87.33)**2) / 54.8733) / 13.1297 )

x = np.array([50,64,78,92,106,120])

px = np.vectorize(pxfunc)(x)

pyxx = np.multiply(pyx,px)

print("pyxx is ", pyxx)

py = np.sum(pyxx)

def func(x):
    return (pyxx[x]/py)

pxy = np.vectorize(func)(np.array([0,1,2,3,4,5]))

xbar = np.sum(np.multiply(x,pxy))

print("pxy is ", pxy)

print("xbar is ", xbar)

xdiff = x - xbar

var = np.sum(np.multiply(xdiff**2,pxy))

std = var**.5

print("var is ", var)

print("std is ", std)