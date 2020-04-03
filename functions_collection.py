# percent overshoot to theta angle  and zeta ratio 
# calculator

# numpy is weird it has ln has a log for some reason
from numpy import log as ln
from math import atan, sqrt, pi, degrees
from sympy import symbols, simplify, fraction,solve, pprint, evalf, im, I
from math import isclose


'''takes overshoot (not percentage! ) and gets zeta needed
to meet that overshoot requirement, ex if overshoot is 20% 
input 0.2 to the function    and it will output the needed damping ratio to meet the os '''
def zeta_from_os(os):
    zeta = -ln(os)/sqrt(pi**2+(ln(os))**2)
    return zeta


'''the angle needed to meet the zeta condition, drawn from origin to + pole, and by symmetry can be extended to bottom angle
taken by drawing a line from the origin to the complex conjugate pole ie the -pole'''
def theta_from_zeta(zeta):
    x = sqrt(1-zeta**2)/zeta
    angle = atan(x)
    return angle

def degree_angle_from_os(os):
    return(degrees(theta_from_zeta(zeta_from_os(os))))

'''answer is in radins'''
def angle_from_os(os):
    return(theta_from_zeta(zeta_from_os(os)))

'''If you have a feedback, please give the G(s)*H(s) for the open_loop_gain parameter'''
def PD_controller_gains(desired_pole, open_loop_tf):
    kp, kd, s = symbols('kp kd s')
    PD_controller_tf = kp +kd*s
    gain_product = PD_controller_tf*open_loop_tf
    closed_tf =  get_closed_loop_tf(gain_product)
    numerator, char_eq = fraction(simplify(closed_tf))
    find_kp(desired_pole, char_eq.subs(kd, 0))

def get_closed_loop_tf(open_loop_tf, feedback=1):
    return open_loop_tf/(open_loop_tf*feedback +1)

def evalute_with_kp_kd(closed_tf):
        pprint(simplify(closed_tf.subs({kd:'44.4', kp:'381.7287'}).evalf()))


''' gets a closed loop tf with kd = 0, it then iterates and find the value of kp that
     makes kp = the desired imaginary part of pole '''
def find_kp(desired_pole, closed_loop_char_equation):
    kp = symbols('kp')
    desired_complex_part = im(desired_pole)
    kp_val = 0
    for i in range(100):
        eq = solve(closed_loop_char_equation.subs(kp, i))
        if(len(eq) == 2):
            current_complex_part = im(eq[1]).evalf()
            
            if(isclose(desired_complex_part, current_complex_part, rel_tol=1e-02, abs_tol=0.17)):
                kp_val = i
                print(str(kp_val) + " found kp")
                print(str(current_complex_part)+ " the im(pole) found with that kp")
                print(str(desired_complex_part)+ " the desired kp")
                return kp_val

    

# example usage
s = symbols('s')
# g = 1/(s**2+s)
Go = 1/(27.77*s**2-296.7525)

pole = 0.8 + 1.557*I
# a = PD_controller_gains(pole,Go)
# print(a)
print(zeta_from_os(0.2))
# pprint(a, use_unicode = False)kp_val
