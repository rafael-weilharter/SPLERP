import numpy as np
import matplotlib.pyplot as plt


class CtrlPoint:
    def __init__(self, x_i = 0, y_i = 0):
        self.x_i = x_i
        self.y_i = y_i
    def getPoint(self):
        return (self.x_i, self.y_i)
    def __str__(self):
        return str(self.getPoint())
    def x(self):
        return self.x_i
    def y(self):
        return self.y_i
    
class CtrlPoint3D:
    def __init__(self, xi = 0, yi = 0, zi = 0):
        self.xi = xi
        self.yi = yi
        self.zi = zi
    def getPoint(self):
        return (self.xi, self.yi, self.zi)
    def __str__(self):
        return str(self.getPoint())
    def x(self):
        return self.xi
    def y(self):
        return self.yi
    def z(self):
        return self.zi

def getPoints3d(anchorX,anchorY,anchorZ):
    # Create dummy variables so params aren't modified
    # We need to keep the original list of anchors in tact
    xvals = anchorX[:]
    yvals = anchorY[:]
    zvals = anchorZ[:]

    pointnumber = len(xvals)
    n = pointnumber - 1
    index = 2
    while index <= len(xvals)-1:
        xvals.insert(index, 0)
        yvals.insert(index, 0)
        zvals.insert(index, 0)
        index+= 2
    for i in range(1,len(xvals)-1):
        xvals[i] *=2
        yvals[i] *=2
        zvals[i] *=2
    def bigmatrix(pointnumber):
        n = pointnumber -1
        matrix = np.zeros((2*n,2*n))
        matrix[0,0:2] = [2,-1]
        matrix[-1,2*n-2], matrix[-1, 2*n-1] = -1, 2
        c1coefficients = [1, 1]
        c2coefficients = [1, -2, 2, -1]
        c1columnstart = 1
        rows = 1
        while rows in range(1,2*n-2):
            matrix[rows, c1columnstart:c1columnstart + 2] = c1coefficients
            c1columnstart += 2
            rows += 2
        morerows = 2
        c2columnstart = 0
        while morerows in range(2,2*n-1):
            matrix[morerows, c2columnstart:c2columnstart+4] = c2coefficients
            c2columnstart += 2
            morerows +=2
        return matrix
    xcontrols = np.linalg.solve(bigmatrix(pointnumber),xvals)
    ycontrols = np.linalg.solve(bigmatrix(pointnumber),yvals)
    zcontrols = np.linalg.solve(bigmatrix(pointnumber),zvals)

    controls = []
    for i in range(len(xcontrols)):
      controls.append(CtrlPoint3D(xcontrols[i], ycontrols[i], zcontrols[i]))

    return controls

def spliner3d(anchorX, anchorY, anchorZ, C, t):

  # C contains the intermediate control points that are inbetween the anchors
  # t contains the t values of the spline, should be [0,1]
  splines = []
  c_index = 0

  # given n anchors, there are n - 1 splines
  for i in range(len(anchorX) - 1):

    Bx = anchorX[i] * (1-t)**3 + 3 * C[c_index].x() * t * (1-t)**2 + 3*C[c_index + 1].x() * (t**2) * (1 - t) + anchorX[i + 1]*t**3
    By = anchorY[i] * (1-t)**3 + 3 * C[c_index].y() * t * (1-t)**2 + 3*C[c_index + 1].y() * (t**2) * (1 - t) + anchorY[i + 1]*t**3
    Bz = anchorZ[i] * (1-t)**3 + 3 * C[c_index].z() * t * (1-t)**2 + 3*C[c_index + 1].z() * (t**2) * (1 - t) + anchorZ[i + 1]*t**3
    c_index = c_index + 2

    # Add the x and y points for a single spline as a tuple
    splines.append( (Bx, By, Bz) )

  return splines 


# Example

# anchorX = [0, -5, -5, 0]
# anchorY = [0, 9, -5, 4]
# anchorZ = [0, 3, -5, 9]
  
# controls = getPoints3d(anchorX, anchorY, anchorZ)
# t = np.linspace(0, 1, 60)
# splines = spliner3d(anchorX, anchorY, anchorZ, controls, t)

# fig = plt.figure()
# ax = fig.add_subplot(projection='3d')
# ax.set_xlabel('x')
# ax.set_ylabel('y')
# ax.set_zlabel('z')
# for i in range(len(splines)):
#     # print("splines: ", splines[i])
#     Bix, Biy, Biz = splines[i]
#     plt.plot(Bix, Biy, Biz)
#     plt.plot(Bix[30], Biy[30], Biz[30], marker="o")

# for j in range(len(anchorX)):
#     plt.plot(anchorX[j], anchorY[j], anchorZ[j], marker="x")

# plt.show()