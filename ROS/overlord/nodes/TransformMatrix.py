#VectorTransform.py

from math import *

class Axis:
    X = 0
    Y = 1
    Z = 2

class TransformMatrix:
    def __init__(self, mat = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]):
        self.m = mat
        #self.show()

    def getTM(self):
        return self.m

    def setTM(self, mat):
        self.m = mat

    def _deg2rad(self, angle_in_deg):
        return angle_in_deg*pi/180.0

    def _rotX(self, x):
        mat = [[ 1 ,  0   ,    0    , 0 ],
               [ 0 ,cos(x),-1*sin(x), 0 ],
               [ 0 ,sin(x),  cos(x) , 0 ],
               [ 0 ,  0   ,    0    , 1 ]]
        return mat
        
    def _rotY(self, x):
        mat = [[  cos(x) , 0 ,sin(x), 0 ],
               [    0    , 1 ,  0   , 0 ],
               [-1*sin(x), 0 ,cos(x), 0 ],
               [    0    , 0 ,  0   , 1 ]]
        return mat

    def _rotZ(self, x):
        mat = [[cos(x),-1*sin(x), 0 , 0 ],
               [sin(x),  cos(x) , 0 , 0 ],
               [  0   ,    0    , 1 , 0 ],
               [  0   ,    0    , 0 , 1 ]]
        return mat

    def _matMult(self, A, B):
        row = []
        mat = []
        for i in range(len(A)):
            for j in range(len(B[0])):
                sumprod = 0
                for k in range(len(B)):
                    sumprod += A[i][k]*B[k][j]
                row.append(sumprod)
            mat.append(row)
            row = []
        return mat
    
    def _preMult(self, mat):
        return self._matMult(mat, self.m)
    
    def _postMult(self, mat):
        return self._matMult(self.m, mat)
        
    def rotate(self, axis, angle, degrees = False):
        angle = float(angle)
        if(degrees): angle = self._deg2rad(angle)
        
        if  (axis == Axis.X): mat = self._rotX(angle)
        elif(axis == Axis.Y): mat = self._rotY(angle)
        elif(axis == Axis.Z): mat = self._rotZ(angle)
        else:
            print("[TransformMatrix.py]:rotate(): INVALID AXIS USED!")
            return None

        self.m = self._preMult(mat)
        #self.show()

    def translate(self, x, y, z):
        mat = [[ 1 , 0 , 0 , float(x) ],
               [ 0 , 1 , 0 , float(y) ],
               [ 0 , 0 , 1 , float(z) ],
               [ 0 , 0 , 0 ,     1    ]]
        self.m = self._preMult(mat)
        #self.show()

    def transformVector(self, xcomp = 0, ycomp = 0, zcomp = 0):
        vect = [[float(xcomp)],
                [float(ycomp)],
                [float(zcomp)],
                [      1     ]]

        newVect = self._postMult(vect)
        return newVect[0][0], newVect[1][0], newVect[2][0]
    
    def show(self):
        rows = len(self.m)
        cols = len(self.m[0])
        for i in range(rows):
            row = '|\t'
            for j in range(cols):
                row += str(round(self.m[i][j], 3)) + '\t'
            print(row + ' |')
