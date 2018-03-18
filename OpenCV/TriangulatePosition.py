#TriangulatePosition.py
import CosineLawSolver
from math import pi

PI_OVER_2 = pi/2

HELP_TEXT = (
"""
    It is expect that the trianlge abc is where:
        -'a' is the width of the symbol the SmartWalker is looking at
        -'b' is the length from the SmartWalker to the left side of the symbol
        -'c' is the length from the SmartWalker to the right side of the symbol
    It is expected that the angles A, B, and C, are in rads
"""
)

def _calcTheta(A_over_2, B, C):
    """
    Arguements:
        A = Angle adjacent to the SmartWalker (opposite side 'a')
        B = Angle at the right side of the symbol (opposite side 'b')
        C = Angle at the left side of the symbol (opposite side 'c')

    Returns:
        theta = The angle the SmartWalker needs to move in rads
    """
    try:
        if (C >= B):
            if (C <= PI_OVER_2): theta = PI_OVER_2 - B - A_over_2
            else:                theta = C - PI_OVER_2 + A_over_2
        else:
            if (B <= PI_OVER_2): theta = -1*(PI_OVER_2 - C - A_over_2)
            else:                theta = -1*(B - PI_OVER_2 + A_over_2)

            
        print ("TriangulatePosition._calcTheta(): Found theta.")
        return theta
    except:
        print ("TriangulatePosition._calcTheta(): No theta found!")
        return None

def calcTargetPose(symbol_width, dist_to_left_side, dist_to_right_side):
    """
    NOTE:
        This function assumes the camera center is superimposed on the
        symbol's center
    """
    try:
        
        #Solve for all triangle values using the given side lengths
        CLS = CosineLawSolver.CosineLawSolver()
        T1 = CLS.solve(a=symbol_width, b=dist_to_left_side, c=dist_to_right_side)

        #Calculate the theta value
        A_over_2 = 0.5*T1.get_A()
        B = T1.get_B()
        C = T1.get_C()
        theta = _calcTheta(A_over_2, B, C)

        #Calculate x (The straight line distance to the symbol
        T2 = CLS.solve(A = A_over_2, a = 0.5*T1.get_a(), b = T1.get_b(), C = C)
        r = T2.get_c()

        print ("TriangulatePosition.calcTargetPose(): Found target pose.")
        return r, theta
    except:
        print ("TriangulatePosition.calcTargetPose(): No target pose found!")
        return None, None
    






