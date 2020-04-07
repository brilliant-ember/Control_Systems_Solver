s = tf('s')
Go = 1/(s**2-s)
# assert that by useing syms funciton syms to veryfy den and num r the same
symbolic_frac = tf_to_symbolic_fraction(Go)
pprint(symbolic_frac)

Go = 1/( (s+1) *(s+2) *(s+10))
q = find_P_controller_gain(3.93,Go)
# assert that K is 164.9
