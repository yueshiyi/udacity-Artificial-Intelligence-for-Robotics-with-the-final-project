
from robot import *
import numpy as np
import matplotlib.pyplot as plt
from scipy import optimize
from math import *

target = robot(0.0, 10.0, 0.0, 2*pi / 30, 1.5)
measurement_noise = 2.0*target.distance # VERY NOISY!!
target.set_noise(0.0, 0.0, measurement_noise)

points=[]
pointx=[]
pointy=[]
for i in range(100):
     target_measurement = target.sense()
     points.append(target_measurement)
     pointx.append(target_measurement[0])
     pointy.append(target_measurement[1])
     target.move_in_circle()


plt.figure()
plt.plot(pointx, pointy, 'r*')
#plt.show()


def circle_fitting(m_points):
    X1 = 0.0
    Y1 = 0.0
    X2 = 0.0
    Y2 = 0.0
    X3 = 0.0
    Y3 = 0.0
    X1Y1 = 0.0
    X1Y2 = 0.0
    X2Y1 = 0.0

    for i in range(len(m_points)):
        X1 = X1 + m_points[i][0]
        Y1 = Y1 + m_points[i][1]
        X2 = X2 + m_points[i][0]* m_points[i][0]
        Y2 = Y2 + m_points[i][1]* m_points[i][1]
        X3 = X3 + m_points[i][0] * m_points[i][0] * m_points[i][0]
        Y3 = Y3 + m_points[i][1] * m_points[i][1] * m_points[i][1]
        X1Y1 = X1Y1 + m_points[i][0] * m_points[i][1]
        X1Y2 = X1Y2 + m_points[i][0]* m_points[i][1] * m_points[i][1]
        X2Y1 = X2Y1 + m_points[i][0] * m_points[i][0] * m_points[i][1]

    N = len(m_points)
    C = N * X2 - X1 * X1
    D = N * X1Y1 - X1 * Y1
    E = N * X3 + N * X1Y2 - (X2 + Y2) * X1
    G = N * Y2 - Y1 * Y1
    H = N * X2Y1 + N * Y3 - (X2 + Y2) * Y1
    a = (H * D - E * G) / (C * G - D * D)
    b = (H * C - E * D) / (D * D - G * C)
    c = -(a * X1 + b * Y1 + X2 + Y2) / N

    A = a / (-2)
    B = b / (-2)
    R = sqrt(a * a + b * b - 4 * c) / 2
    m_fCenterX = A
    m_fCenterY = B
    m_fRadius = R
    return m_fCenterX, m_fCenterY, m_fRadius

cx, cy, r = circle_fitting(points)

theta = np.arange(0, 2*np.pi, 0.01)
x = cx + r * np.cos(theta)
y = cy + r * np.sin(theta)
plt.plot(x, y, 'g-')
plt.show()


