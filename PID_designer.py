# numpy is weird it has ln as log for some reason
from numpy import log as ln, arange, roots
from math import atan, sqrt, pi, degrees
from sympy import symbols, simplify, fraction,solve, pprint, evalf, im, I
from math import isclose
from control import root_locus, tf, tfdata, feedback

#readability note: sym = symbolic 

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

def get_symbolic_characteristic_eq(symbolic_tf):
    num, char_eq = fraction(simplify(symbolic_tf))
    return char_eq

'''takes the desired +im(pole) and optf to get the needed P controller gain to meet that requirement,
 if you have a feedback gain then opn tf is feedback gain * opn loop gain'''
def find_P_controller_gain(positive_cmplx_part_wanted, open_loop_tf, iteration_range = 100, decimal_places=0.01):
    #char eq will always be in the form (Kp+Kds+Ki/s)(opn lp num)+(opn lop den)
    num, den = get_num_den_from_tf(open_loop_tf)
    # we're finding kp first, so kds and ki/s are zero and we loop with different kp
    # and we settel for the one that gets us our im(wanted_pole)
    # char eq = (kp + 0s +0/s)*num + den
    Kp_test_list = arange(0,iteration_range,decimal_places)
    last_best = 1000 # random big number to start with
    for k in Kp_test_list:
        tmp_char_eq = numpy_poly_adder((k*num),den) 
        print({
            "k":k,
            "tmp_char_eq":tmp_char_eq,
        })
        current_im_pole = roots(tmp_char_eq)[0].imag
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

# def compare_current_to_desired(current, desired):
#     if(isclose(desired, current, rel_tol=1e-01, abs_tol=0.17)):
#         current_diff = abs(desired - current)
#         print("current diff: " +str(current_diff))
#         print(last_best)
#         if(current_diff >= last_best ):
#             print(str(k) + " I am the found kp")
#             print(str(wanted_pole) + " desired pole im")
#             print(str(current_cmplx_prt) + "found cmplx part")
#         else:
#             last_best = current_diff

    # sym_tf = tf_to_symbolic_fraction(open_loop_tf)
    # char_eq = get_symbolic_characteristic_eq(sym_tf)
    # # root_locus_points = root_locus(open_loop_tf, plot=False, )[0]
    # solve_with_algebra(char_eq,arange(0,20,0.01),positive_cmplx_part_wanted)

    # root_locus_gains = root_locus(open_loop_tf)[1]
    # for gain in root_locus_gains:
    #     eq_to_solve = char_eq + gain
    #     solved = solve(eq_to_solve)
    #     # if it's not length 2 then there is no im part
    #     if len(solved)==2:
    #         # the positive is the 2nd part(what we want), that's how sympy solves it
    #         current_cmplx_prt = im(solved[1]).evalf()

    #         if(isclose(positive_cmplx_part_wanted,current_cmplx_prt,rel_tol=1e-02, abs_tol=0.17)):
    #             print(str(gain) + " I am the found kp")
    #             print(str(positive_cmplx_part_wanted) + " desired pole im")
    #             print(str(current_cmplx_prt) + "found cmplx part")
                
        # else:
        #     print(str(gain) + " I am the found kp")
        #     print(str(positive_cmplx_part_wanted) + " desired pole im")
        #     print(str(current_cmplx_prt) + "found cmplx part")
        #     # raise Exception("can't find an accurate enough Kp")

#scratched idea, was too slow
# def solve_with_root_locus(open_loop_tf, k_list, wanted_pole):
#     for pnt in root_locus_points:
#         current_cmplx_prt = pnt[1].imag
#         print(current_cmplx_prt)
#         if current_cmplx_prt != 0:
#             if(isclose(positive_cmplx_part_wanted,current_cmplx_prt,rel_tol=1e-02, abs_tol=0.17)):
#                 print(str(gain) + " I am the found kp")
#                 print(str(positive_cmplx_part_wanted) + " desired pole im")
#                 print(str(current_cmplx_prt) + "found cmplx part")

#scratched idea takes too long
# def solve_with_algebra(char_eq, k_list, wanted_pole):
#     pprint(char_eq)
#     last_best = 1000 # random big number to start with
#     for k in k_list:
#         new_char_eq = char_eq + k
#         solved = solve(new_char_eq)
#         # if it's not length 2 then there is no im part
#         if len(solved)==2:
#             # the positive is the 2nd part(what we want), that's how sympy solves it
#             current_cmplx_prt = im(solved[1]).evalf()
#             if(isclose(wanted_pole,current_cmplx_prt,rel_tol=1e-01, abs_tol=0.17)):
#                 current_diff = abs(wanted_pole - current_cmplx_prt)
#                 print("current diff: " +str(current_diff))
#                 print(last_best)
#                 if(current_diff >= last_best ):
#                     print(str(k) + " I am the found kp")
#                     print(str(wanted_pole) + " desired pole im")
#                     print(str(current_cmplx_prt) + "found cmplx part")
#                     break
#                 else:
#                     last_best = current_diff
    


##### Tests####
s = tf('s')
Go = 1/(s**2+s)
# assert that by useing syms funciton syms to veryfy den and num r the same
symbolic_frac = tf_to_symbolic_fraction(Go)

find_P_controller_gain(3.89,Go)

