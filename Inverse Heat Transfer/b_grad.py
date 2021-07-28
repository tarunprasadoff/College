import numpy as np
from math import exp as exp

def func(x):
    return ( (75.82178 * exp(-28.1853 / x)) / (x**.5) )

print(np.vectorize(func)(np.array([60,81,99,118,142,157])))
