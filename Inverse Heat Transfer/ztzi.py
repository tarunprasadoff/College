import numpy as np

zt = np.array([[.66749,.5957,.54933,.51042,.47125,.45080], [6.11936483, 5.9488287,  5.73235053, 5.49690276, 5.21732088, 5.0568233]])

z = np.transpose(zt)

ztz = np.matmul(zt,z)
ztzi = np.linalg.inv(ztz)

print("ztz is ", ztz)
print("")
print("ztzi is ", ztzi)