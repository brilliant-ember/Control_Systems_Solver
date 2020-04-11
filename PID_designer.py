# numpy is weird it has ln as log for some reason
from numpy import log as ln, arange, roots
from math import atan, sqrt, pi, degrees, tan
from sympy import symbols, simplify, fraction,solve, pprint, evalf, im, I
from math import isclose
from control import root_locus, tf, tfdata, feedback

#readability note: sym = symbolic
# This is based on 2nd order approximation, if you try to use a 3rd order tf, it will give you
# the 2nd approx, if u input a real 2nd order tf it will give u a true result 

def get_num_den_from_tf(tf):
    x = symbols('x')
    num, den = tfdata(tf)
    num = num[0][0]
    den = den[0][0]
    return(num,den)

def tf_to_symbolic_fraction(tf):
    x = symbols('x')
    num, den = tfdata(tf)
    num = num[0][0]
    den = den[0][0]
    counter = 1
    length_num = len(num)
    length_den = len(den)

    sym_num, sym_den = 0,0

    if(length_num == 0):
        raise ValueError ("The fraction num should not be empty")
    elif(length_den == 0 ):
        raise ValueError("The fraction den should not be empty")

    for i in range(length_num):
        sym_num+=num[i]*(x**(length_num - counter))
        counter += counter
    counter = 1
    for i in range(length_den):
        sym_den+=den[i]*(x**(length_den - counter))
        counter += counter
    return sym_num/sym_den

def symbolic_fraction_to_tf(sym_frac):
    num , den = fraction(sym_frac)
    pass

def pick_positive_im_pole(pole_array):
    for i in pole_array:
        x = i.imag
        if x > 0:
            return x
    return 0
def get_symbolic_characteristic_eq(symbolic_tf):
    num, char_eq = fraction(simplify(symbolic_tf))
    return char_eq

'''takes the desired +im(pole) and optf to get the needed P controller gain to meet that requirement,
 if you have a feedback gain then opn tf is feedback gain * opn loop gain'''
def find_P_controller_gain(positive_cmplx_part_wanted, open_loop_tf, iteration_range = 300, decimal_places=0.01):
    #char eq will always be in the form (Kp+Kds+Ki/s)(opn lp num)+(opn lop den)
    num, den = get_num_den_from_tf(open_loop_tf)
    # we're finding kp first, so kds and ki/s are zero and we loop with different kp
    # and we settel for the one that gets us our im(wanted_pole)
    # char eq = (kp + 0s +0/s)*num + den
    Kp_test_list = arange(0,iteration_range,decimal_places)
    last_best = 1000 # random big number to start with
    for k in Kp_test_list:
        #char eq is always in the form den + H(s)*num where den and num are that of the opn loop tf
        tmp_char_eq = numpy_poly_adder((k*num),den) 
        
        current_im_pole = pick_positive_im_pole(roots(tmp_char_eq))

        print({
            "k":k,
            "tmp_char_eq":tmp_char_eq,
            "current pole":current_im_pole
        })
        if( isclose(positive_cmplx_part_wanted, current_im_pole, rel_tol=1e-01, abs_tol=0.17) ):
            current_diff = abs(current_im_pole - positive_cmplx_part_wanted)
            if(current_diff >= last_best ):
                print(str(k) + " I am the found kp")
                print(str(current_im_pole) + "found cmplx part")
                print({
                    "tmpCharEq":tmp_char_eq,
                })
                break
            else:
                last_best = current_diff
'''numpy polys are in the form [1,4,7] which is x^2 + 4x +7, so adding 
in order is required to add polys for example [1,4,7] + [3,2] = [1,7,9]'''
def numpy_poly_adder(poly1,poly2):
    l1, l2 = len(poly1),len(poly2)
    if l1 ==0 or l2 == 0:
        raise ValueError("none of the polynomials can be length 0")
    if l1 > l2:
        output = poly1.copy()
        for i in reversed(range(l2)):
            output[l1-1-i]+=poly2[l2-1-i]
        return output
    else:
        output = poly2.copy()
        for i in reversed(range(l1)):
            output[l2-1-i]+=poly1[l1-1-i]
    return output  


'''takes overshoot (not percentage! ) and gets zeta needed
to meet that overshoot requirement, ex if overshoot is 20% 
input 0.2 to the function    and it will output the needed damping ratio to meet the os '''
def damping_ratio_from_os(os):
    zeta = -ln(os)/sqrt(pi**2+(ln(os))**2)
    return zeta


'''the angle needed to meet the zeta condition, drawn from origin to + pole, and by symmetry can be extended to bottom angle
taken by drawing a line from the origin to the complex conjugate pole ie the -pole'''
def angle_from_damping_ratio(zeta):
    x = sqrt(1-zeta**2)/zeta
    angle = atan(x)
    return angle

def degree_angle_from_os(os):
    return(degrees(angle_from_damping_ratio(damping_ratio_from_os(os))))

'''answer is in radins'''
def angle_from_os(os):
    return(angle_from_damping_ratio(damping_ratio_from_os(os)))

##### Tests####
s = tf('s')
Go = 1/( (s+1) *(s+2) *(s+10))
# # assert that by useing syms funciton syms to veryfy den and num r the same
# symbolic_frac = tf_to_symbolic_fraction(Go)

# q = find_P_controller_gain(3.93,Go)

a = tan(angle_from_os(0.2))
print(a)

