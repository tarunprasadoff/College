import numpy as np

ztzi = np.array([[109.12526953, -10.59504761], [-10.59504761,   1.03397981]])
zte = np.array([3.52321543, 36.23492377])

print("da is ", np.matmul(ztzi,zte))