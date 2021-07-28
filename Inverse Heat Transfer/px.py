import numpy as np
from math import exp as exp

def func(x):
    return ( exp(-((x-87.33)**2) / 54.8733) / 13.1297 )

print(np.vectorize(func)(np.array([50,64,78,92,106,120])))