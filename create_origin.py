from sympy.solvers import solve
from sympy import Symbol
# #rvec1 = [['left',
#   0.2696336000000001,
#   [0.3658271735916299,
#    0.8750368926621974,
#    0.4636149892895105,
#    -1.4827321549017185,
#    0.61291888063056,
#    0.6150745470758807]],
# # ['top',
#   0.3868809000000001,
#   [0.6354595926492418,
#    0.8750351455883683,
#    0.8505026661846955,
#    -0.7304766335882252,
#    1.7598406965231221,
#    1.7609341207754496]]]
#
# #rvec2 =[['left',
#   0.1688677999999997,
#   [0.3665822210068477,
#    0.8623968985934857,
#    0.5636172285703518,
#    -1.4827511409532708,
#    0.6129670771707507,
#    0.6150641804044209]],
#  ['top',
#   0.29790969999999994,
#   [0.5354394592538747,
#    0.862396020188954,
#    0.8615200726422626,
#    -0.730507665641822,
#    1.7598965571839342,
#    1.760917635394325]]]

#coordinates in [X,Z]
#Side coordinates [z-formula]
# coor1 = [tvec1[0][2][0],tvec1[0][2][2]] #[X,Z]
# coor2 = [tvec2[0][2][0],tvec2[0][2][2]]
#
# #TOP Coordinates [x-formula]
# coor3 = [rvec1[1][2][0],rvec1[1][2][2]]
# coor4 = [rvec2[1][2][0],rvec2[1][2][2]]
#

#Formula is z = ax+b
#slope a = (z2-z1)/(z2-z1)
def formula_line(orientation, coor1, coor2):
    if orientation == 'x':
        ax = (coor2[1]-coor1[1])/(coor2[0]-coor1[0])
        bx = coor1[1]-ax*coor1[0]
        global line_x
        line_x = [ax, bx] #y=ax*x+bx
        print('print x', line_x)
        return line_x

    if orientation == 'z':
        az = (coor2[1] - coor1[1]) / (coor2[0] - coor1[0])
        bz = coor1[1] - az * coor1[0]
        global line_z
        line_z = [az, bz] #y=az*x+bz
        print('print z', line_z)
        return line_z


def intersect_lines(line_x,line_z):
    x = Symbol('x')
    a = (line_x[0]-line_z[0])
    b = line_x[1]-line_z[1]
    x = solve(a*x+b,x)
    x = float(x[0])
    y = Symbol('y')
    t = float(line_x[0])
    ax = t*x
    y = solve(y-ax-line_x[1])
    y = float(y[0])

    global origin
    origin = [x,y]
    return origin


# COPY stuff
# create_origin.formula_line('x',coor1,coor2)
# create_origin.formula_line('z',coor3,coor4)
# create_origin.intersect_lines(line_x,line_z)
# print(origin) #[X,Z]