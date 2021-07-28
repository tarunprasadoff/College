import numpy as np

def func(x):
    return ( 5.3089 / (x**.5) )

print(np.vectorize(func)(np.array([60,81,99,118,142,157])))