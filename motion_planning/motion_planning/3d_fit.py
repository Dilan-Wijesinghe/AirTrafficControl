import numpy as np
import matplotlib.pyplot as plt
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression

# Point cloud get from cv

# data = np.array([[ 1 ,  1 ,  1 ],[ 2. ,  2.  , 3],[ 3. ,  3.  , 4], [ 4,   4 , 5],
#                  [ 5,  5, 6], [ 6,  6 , 7], [7, 7, 8],[ 8,   8, 9],
#                  [ 9,  9, 10], [ 10,  10, 11]])
# x = data[:,0]
# y = data[:,1]
# z = data[:,2]
z = np.linspace(0, 15, 10)
x = np.sin(z)
y = np.cos(z)

# x = np.array(x)
# y = np.array(y)
# z = np.array(z)

data_xy = np.array([x,y])
data_yz = np.array([y,z])
data_xz = np.array([x,z])

poly = PolynomialFeatures(degree=3)
X_t = poly.fit_transform(data_xy.transpose())
Y_t = poly.fit_transform(data_yz.transpose())
Z_t = poly.fit_transform(data_xz.transpose())

clf = LinearRegression()
clf.fit(X_t, z)
z_pred = clf.predict(X_t)
clf.fit(Y_t, x)
x_pred = clf.predict(Y_t)
clf.fit(Z_t, y)
y_pred = clf.predict(Z_t)

# print(z)
print(z_pred)
# print(x)
print(x_pred)
# print(y)
print(y_pred)


fig = plt.figure()
ax = plt.subplot(projection='3d')
#red line is the predit data trajectory
ax.plot(x_pred, y_pred, z_pred, 'r')
#blue dot is the original input data
ax.scatter(x, y, z)
plt.show()

