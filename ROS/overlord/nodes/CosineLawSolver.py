#CosineLawSolver.py
import math

HELP_TEXT = (
"""

NOTES:
        1) a, b, and c, are three sides that construct a triangle.
        2) A, B, and C, are the angles opposite to the side length named
            after their lower case couterpart. e.g. The Angle 'A' is
            opposite the side length 'a'.

CosineLawSolver.solve()
-----------------------

    Arguements:
        A = Angle A in rads
        B = Angle B in rads
        C = Angle C in rads
        a = side length a
        b = side length b
        c = side length c
        degrees = True/False
            (If True interprets A, B, C as angles in degrees instead of rads)

    Returns:
        A Triangle Object.

Triangle.get_x()
----------------

    Returns:
        The value specified by 'x' in the function name. If
        CosineLawSolver.solve() did not find a solution these
        methods will return 'None'.

Triangle.getTriangle()
----------------------

    Returns:
        An array containing the angles and side lengths in the form:
        [A, B, C, a, b, c]
        Where the variables represent the same quantities as the
        arguements for the CosineLawSolver.solve() method
"""
)

POSSIBLE_INPUT_ERRORS = (
"""
COSINELAWSOLVER 
Triangle was deemed impossible given the inputs. Please ensure that
the following is true regarding the inputs of the triangle:
1) At least three sides are known or two sides and the angle between them.
2) If three sides are known, no one given side is larger than the sum of the other two.
"""
)

class CosineLawSolver:
    
    ##====================CONSTRUCTOR====================##
    
    def __init__(self):
        self._DEG2RAD = math.pi/180.0
        self._has_A = False
        self._has_B = False
        self._has_C = False
        self._has_a = False
        self._has_b = False
        self._has_c = False
        self._iterations = 0

    ##====================PRIVATE=METHODS====================##

    def _reset(self):
        self._has_A = False
        self._has_B = False
        self._has_C = False
        self._has_a = False
        self._has_b = False
        self._has_c = False
        self._iterations = 0
        
    def _checkValues(self, A, B, C, a, b, c):
        self._has_A = not(A is None)
        self._has_B = not(B is None)
        self._has_C = not(C is None)
        self._has_a = not(a is None)
        self._has_b = not(b is None)
        self._has_c = not(c is None)

    def _solutionExists(self, A, B, C, a, b, c):
        self._checkValues(A, B, C, a, b, c)
        return ((self._has_a and self._has_b and self._has_c) or
                (self._has_a and self._has_b and self._has_C) or
                (self._has_b and self._has_c and self._has_A) or
                (self._has_c and self._has_a and self._has_B))

    def _calc_A(self, A, B, C, a, b, c):
        try:
            result = math.acos(((b**2)+(c**2)-(a**2))/(2*b*c))
            self._has_A = True
            return result, "No Error"
        except Exception as e:
            return None, "Error calcualting A: " + str(e)

    def _calc_B(self, A, B, C, a, b, c):
        try:
            result = math.acos(((a**2)+(c**2)-(b**2))/(2*a*c))
            self._has_B = True
            return result, "No Error"
        except Exception as e:
            return None, "Error calcualting B: " + str(e)

    def _calc_C(self, A, B, C, a, b, c):
        try:
            result = math.acos(((b**2)+(a**2)-(c**2))/(2*b*a))
            self._has_C = True
            return result, "No Error"
        except Exception as e:
            return None, "Error calcualting C: " + str(e)

    def _calc_a(self, A, B, C, a, b, c):
        try:
            result = math.sqrt((b**2)+(c**2)-2*b*c*math.cos(A))
            self._has_a = True
            return result, "No Error"
        except Exception as e:
            return None, "Error calcualting a: " + str(e)

    def _calc_b(self, A, B, C, a, b, c):
        try:
            result = math.sqrt((a**2)+(c**2)-2*a*c*math.cos(B))
            self._has_b = True
            return result, "No Error"
        except Exception as e:
            return None, "Error calcualting b: " + str(e)

    def _calc_c(self, A, B, C, a, b, c):
        try:
            result = math.sqrt((b**2)+(a**2)-2*b*a*math.cos(C))
            self._has_c = True
            return result, "No Error"
        except Exception as e:
            return None, "Error calculating c: " + str(e)
    
    def _makeInputsFloat(self, A, B, C, a, b, c):
        if(self._has_A): A = float(A)
        if(self._has_B): B = float(B)
        if(self._has_C): C = float(C)
        if(self._has_a): a = float(a)
        if(self._has_b): b = float(b)
        if(self._has_c): c = float(c)
        return A, B, C, a, b, c                

    def _getSolution(self, A, B, C, a, b, c):
        Err = "No Error"
        self._checkValues(A, B, C, a, b, c)
        A, B, C, a, b, c = self._makeInputsFloat(A, B, C, a, b, c)

        print ("[CosineLawSolver.py]:_getSolution(): Showing iterative data")
        print ("  A=%s | B=%s | C=%s | a=%s | b=%s | c=%s | Status: %s" % (A,B,C,a,b,c,Err))
        while(not(self._has_A and self._has_B and self._has_C
                and self._has_a and self._has_b and self._has_c)):

            self._iterations += 1
            if(self._iterations > 3):
                self._iterations = 0
                has_all_sides = self._has_a and self._has_b and self._has_c
                if(has_all_sides and ((a > b + c) or (b > a + c) or (c > a + b))):
                    print (POSSIBLE_INPUT_ERRORS)
                else:
                    print ("Uknown Error occured in the CosineLawSolver")
                return None
            
            
            if(not(self._has_A)): A, Err = self._calc_A(A, B, C, a, b, c)
            if(not(self._has_B)): B, Err = self._calc_B(A, B, C, a, b, c)
            if(not(self._has_C)): C, Err = self._calc_C(A, B, C, a, b, c)
            if(not(self._has_a)): a, Err = self._calc_a(A, B, C, a, b, c)
            if(not(self._has_b)): b, Err = self._calc_b(A, B, C, a, b, c)
            if(not(self._has_c)): c, Err = self._calc_c(A, B, C, a, b, c)

        print ("  A=%s | B=%s | C=%s | a=%s | b=%s | c=%s | Status: %s" % (A,B,C,a,b,c,Err))
            
        self._iterations = 0
        return [A, B, C, a, b, c]

    ##====================PUBLIC=METHODS====================##
    
    def solve(self, A=None, B=None, C=None, a=None, b=None, c=None, degrees=False):
        if(self._solutionExists(A, B, C, a, b, c)):
            
            if(degrees):
                if(A is not(None)): A *= self._DEG2RAD
                if(B is not(None)): B *= self._DEG2RAD
                if(C is not(None)): C *= self._DEG2RAD
                if(a is not(None)): a *= self._DEG2RAD
                if(b is not(None)): b *= self._DEG2RAD
                if(c is not(None)): c *= self._DEG2RAD

                solution = self._getSolution(A, B, C, a, b, c)

                if(not(solution is None)):
                    solution[0] /= self._DEG2RAD
                    solution[1] /= self._DEG2RAD
                    solution[2] /= self._DEG2RAD
                    solution[3] /= self._DEG2RAD
                    solution[4] /= self._DEG2RAD
                    solution[5] /= self._DEG2RAD

                return Triangle(solution)

            else:
                return Triangle(self._getSolution(A, B, C, a, b, c))

            self._reset()

        else:
            print ("\n[CosineLawSolver.py]: NOT ENOUGH INFORMATION GIVEN FOR SOLUTION!\n")
            print ("\tUsage give below:"+HELP_TEXT+"\n\n")
            
##===============TRIANGLE=CLASS===============##
class Triangle():
    def __init__(self, cosine_law_solver_triangle_array):
        self.triangle = cosine_law_solver_triangle_array
        self.A = self.triangle[0]
        self.B = self.triangle[1]
        self.C = self.triangle[2]
        self.a = self.triangle[3]
        self.b = self.triangle[4]
        self.c = self.triangle[5]

    def get_A(self): return self.A
    def get_B(self): return self.B
    def get_C(self): return self.C
    def get_a(self): return self.a
    def get_b(self): return self.b
    def get_c(self): return self.c
    def getTriangle(self): return self.triangle
    
    
