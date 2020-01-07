from sympy.vector import CoordSys3D, Del, vector
from sympy import symbols
from sympy import sqrt, ImmutableDenseMatrix, cos, sin
from sympy import Dummy
from sympy.core import Expr

class DummyVariable(Dummy):
    def __new__(cls, name=None, dummy_index=None, value=0, **assumptions):
        obj = Dummy.__new__(cls, name, dummy_index, **assumptions)
        obj.value = 0.0
        return obj

    @staticmethod
    def eval_all(expr: Expr):
        all_sym = expr.free_symbols
        replacements = []
        for sym in all_sym:
            if isinstance(sym, DummyVariable):
                replacements.append((sym, sym.value))
        if len(replacements):
            expr = expr.subs(replacements)
        return expr


class CoordSysVariable(CoordSys3D):
    def __new__(cls, name, base: CoordSys3D):
        x, y, z = symbols('x,y,z', cls=DummyVariable)

        q, r, s = symbols('q,r,s', cls=DummyVariable)

        tvec: vector.VectorAdd = x * base.i + y * base.j + z * base.k
        rvec: vector.VectorAdd = q * base.i + r * base.j + s * base.k

        rot_mat = ImmutableDenseMatrix([
        [                                                                            (q**2 + (r**2 + s**2)*cos(sqrt(q**2 + r**2 + s**2)))/(q**2 + r**2 + s**2),  (q*r*(1 - cos(sqrt(q**2 + r**2 + s**2)))*sqrt(q**2 + r**2 + s**2) + s*(q**2 + r**2 + s**2)*sin(sqrt(q**2 + r**2 + s**2)))/(q**2 + r**2 + s**2)**(3/2), (q*s*(1 - cos(sqrt(q**2 + r**2 + s**2)))*sqrt(q**2 + r**2 + s**2) - r*(q**2 + r**2 + s**2)*sin(sqrt(q**2 + r**2 + s**2)))/(q**2 + r**2 + s**2)**(3/2)],
        [(q*r*(1 - cos(sqrt(q**2 + r**2 + s**2)))*sqrt(q**2 + r**2 + s**2) - s*(q**2 + r**2 + s**2)*sin(sqrt(q**2 + r**2 + s**2)))/(q**2 + r**2 + s**2)**(3/2),                                                                              (r**2 + (q**2 + s**2)*cos(sqrt(q**2 + r**2 + s**2)))/(q**2 + r**2 + s**2), (q*(q**2 + r**2 + s**2)*sin(sqrt(q**2 + r**2 + s**2)) + r*s*(1 - cos(sqrt(q**2 + r**2 + s**2)))*sqrt(q**2 + r**2 + s**2))/(q**2 + r**2 + s**2)**(3/2)],
        [(q*s*(1 - cos(sqrt(q**2 + r**2 + s**2)))*sqrt(q**2 + r**2 + s**2) + r*(q**2 + r**2 + s**2)*sin(sqrt(q**2 + r**2 + s**2)))/(q**2 + r**2 + s**2)**(3/2), (-q*(q**2 + r**2 + s**2)*sin(sqrt(q**2 + r**2 + s**2)) + r*s*(1 - cos(sqrt(q**2 + r**2 + s**2)))*sqrt(q**2 + r**2 + s**2))/(q**2 + r**2 + s**2)**(3/2),                                                                             (s**2 + (q**2 + r**2)*cos(sqrt(q**2 + r**2 + s**2)))/(q**2 + r**2 + s**2)]])
        # obj = base.locate_new(name, tvec)
        obj = CoordSys3D.__new__(cls, name, location=tvec,
                                 rotation_matrix=rot_mat,
                                 vector_names=base._vector_names,
                                 variable_names=base._variable_names,
                                 parent=base)
        obj.x, obj.y, obj.z = x, y, z
        obj.q, obj.r, obj.s = q, r, s

        obj._tvec = tvec
        obj._rvec = rvec
        obj._rot_mat = rot_mat
        return obj


    @property
    def tvec(self):
        return self._tvec

    @tvec.setter
    def tvec(self, value):
        self.x.value, self.y.value, self.z.value = value

    @property
    def rvec(self):
        return self._rvec

    @rvec.setter
    def rvec(self, value):
        self.q.value, self.r.value, self.s.value = value

    @property
    def angle(self):
        return self.rvec.magnitude()

    @angle.setter
    def angle(self, value):
        pass

    @property
    def axis(self):
        return self.rvec/self.angle

    @axis.setter
    def axis(self, value):
        pass


if __name__ == "__main__":
    C = CoordSys3D('C')
    delop = Del()
    delop.cross(C.x*C.y*C.z*C.i).doit()
    (delop ^ C.x*C.y*C.z*C.i).doit()

    x, y, z = symbols('x,y,z')
    N = CoordSys3D('N')
    a = 3 * N.i
    type(a)

    v = x*N.i + y*N.j + z*N.k
    type(v)

    D = N.locate_new('D', v)
    D.position_wrt(N)

    E = CoordSysVariable('E', C)
    # F = E.coordsys.locate_new('F', E.coordsys.i*2)
'''
                                                                                                                       q**2/((q**2 + r**2 + s**2)*(q**2/(q**2 + r**2 + s**2) + r**2/(q**2 + r**2 + s**2) + s**2/(q**2 + r**2 + s**2))) + (-q**2/((q**2 + r**2 + s**2)*(q**2/(q**2 + r**2 + s**2) + r**2/(q**2 + r**2 + s**2) + s**2/(q**2 + r**2 + s**2))) + 1)*cos(sqrt(q**2 + r**2 + s**2)), -q*r*cos(sqrt(q**2 + r**2 + s**2))/((q**2 + r**2 + s**2)*(q**2/(q**2 + r**2 + s**2) + r**2/(q**2 + r**2 + s**2) + s**2/(q**2 + r**2 + s**2))) + q*r/((q**2 + r**2 + s**2)*(q**2/(q**2 + r**2 + s**2) + r**2/(q**2 + r**2 + s**2) + s**2/(q**2 + r**2 + s**2))) + s*sin(sqrt(q**2 + r**2 + s**2))/(sqrt(q**2 + r**2 + s**2)*sqrt(q**2/(q**2 + r**2 + s**2) + r**2/(q**2 + r**2 + s**2) + s**2/(q**2 + r**2 + s**2))), -q*s*cos(sqrt(q**2 + r**2 + s**2))/((q**2 + r**2 + s**2)*(q**2/(q**2 + r**2 + s**2) + r**2/(q**2 + r**2 + s**2) + s**2/(q**2 + r**2 + s**2))) + q*s/((q**2 + r**2 + s**2)*(q**2/(q**2 + r**2 + s**2) + r**2/(q**2 + r**2 + s**2) + s**2/(q**2 + r**2 + s**2))) - r*sin(sqrt(q**2 + r**2 + s**2))/(sqrt(q**2 + r**2 + s**2)*sqrt(q**2/(q**2 + r**2 + s**2) + r**2/(q**2 + r**2 + s**2) + s**2/(q**2 + r**2 + s**2)))],
[-q*r*cos(sqrt(q**2 + r**2 + s**2))/((q**2 + r**2 + s**2)*(q**2/(q**2 + r**2 + s**2) + r**2/(q**2 + r**2 + s**2) + s**2/(q**2 + r**2 + s**2))) + q*r/((q**2 + r**2 + s**2)*(q**2/(q**2 + r**2 + s**2) + r**2/(q**2 + r**2 + s**2) + s**2/(q**2 + r**2 + s**2))) - s*sin(sqrt(q**2 + r**2 + s**2))/(sqrt(q**2 + r**2 + s**2)*sqrt(q**2/(q**2 + r**2 + s**2) + r**2/(q**2 + r**2 + s**2) + s**2/(q**2 + r**2 + s**2))),                                                                                                                                              r**2/((q**2 + r**2 + s**2)*(q**2/(q**2 + r**2 + s**2) + r**2/(q**2 + r**2 + s**2) + s**2/(q**2 + r**2 + s**2))) + (-r**2/((q**2 + r**2 + s**2)*(q**2/(q**2 + r**2 + s**2) + r**2/(q**2 + r**2 + s**2) + s**2/(q**2 + r**2 + s**2))) + 1)*cos(sqrt(q**2 + r**2 + s**2)),  q*sin(sqrt(q**2 + r**2 + s**2))/(sqrt(q**2 + r**2 + s**2)*sqrt(q**2/(q**2 + r**2 + s**2) + r**2/(q**2 + r**2 + s**2) + s**2/(q**2 + r**2 + s**2))) - r*s*cos(sqrt(q**2 + r**2 + s**2))/((q**2 + r**2 + s**2)*(q**2/(q**2 + r**2 + s**2) + r**2/(q**2 + r**2 + s**2) + s**2/(q**2 + r**2 + s**2))) + r*s/((q**2 + r**2 + s**2)*(q**2/(q**2 + r**2 + s**2) + r**2/(q**2 + r**2 + s**2) + s**2/(q**2 + r**2 + s**2)))],
[-q*s*cos(sqrt(q**2 + r**2 + s**2))/((q**2 + r**2 + s**2)*(q**2/(q**2 + r**2 + s**2) + r**2/(q**2 + r**2 + s**2) + s**2/(q**2 + r**2 + s**2))) + q*s/((q**2 + r**2 + s**2)*(q**2/(q**2 + r**2 + s**2) + r**2/(q**2 + r**2 + s**2) + s**2/(q**2 + r**2 + s**2))) + r*sin(sqrt(q**2 + r**2 + s**2))/(sqrt(q**2 + r**2 + s**2)*sqrt(q**2/(q**2 + r**2 + s**2) + r**2/(q**2 + r**2 + s**2) + s**2/(q**2 + r**2 + s**2))), -q*sin(sqrt(q**2 + r**2 + s**2))/(sqrt(q**2 + r**2 + s**2)*sqrt(q**2/(q**2 + r**2 + s**2) + r**2/(q**2 + r**2 + s**2) + s**2/(q**2 + r**2 + s**2))) - r*s*cos(sqrt(q**2 + r**2 + s**2))/((q**2 + r**2 + s**2)*(q**2/(q**2 + r**2 + s**2) + r**2/(q**2 + r**2 + s**2) + s**2/(q**2 + r**2 + s**2))) + r*s/((q**2 + r**2 + s**2)*(q**2/(q**2 + r**2 + s**2) + r**2/(q**2 + r**2 + s**2) + s**2/(q**2 + r**2 + s**2))),                                                                                                                                              s**2/((q**2 + r**2 + s**2)*(q**2/(q**2 + r**2 + s**2) + r**2/(q**2 + r**2 + s**2) + s**2/(q**2 + r**2 + s**2))) + (-s**2/((q**2 + r**2 + s**2)*(q**2/(q**2 + r**2 + s**2) + r**2/(q**2 + r**2 + s**2) + s**2/(q**2 + r**2 + s**2))) + 1)*cos(sqrt(q**2 + r**2 + s**2))]])
mat.simplify()
Out[30]: 
Matrix([
[                                                                            (q**2 + (r**2 + s**2)*cos(sqrt(q**2 + r**2 + s**2)))/(q**2 + r**2 + s**2),  (q*r*(1 - cos(sqrt(q**2 + r**2 + s**2)))*sqrt(q**2 + r**2 + s**2) + s*(q**2 + r**2 + s**2)*sin(sqrt(q**2 + r**2 + s**2)))/(q**2 + r**2 + s**2)**(3/2), (q*s*(1 - cos(sqrt(q**2 + r**2 + s**2)))*sqrt(q**2 + r**2 + s**2) - r*(q**2 + r**2 + s**2)*sin(sqrt(q**2 + r**2 + s**2)))/(q**2 + r**2 + s**2)**(3/2)],
[(q*r*(1 - cos(sqrt(q**2 + r**2 + s**2)))*sqrt(q**2 + r**2 + s**2) - s*(q**2 + r**2 + s**2)*sin(sqrt(q**2 + r**2 + s**2)))/(q**2 + r**2 + s**2)**(3/2),                                                                              (r**2 + (q**2 + s**2)*cos(sqrt(q**2 + r**2 + s**2)))/(q**2 + r**2 + s**2), (q*(q**2 + r**2 + s**2)*sin(sqrt(q**2 + r**2 + s**2)) + r*s*(1 - cos(sqrt(q**2 + r**2 + s**2)))*sqrt(q**2 + r**2 + s**2))/(q**2 + r**2 + s**2)**(3/2)],
[(q*s*(1 - cos(sqrt(q**2 + r**2 + s**2)))*sqrt(q**2 + r**2 + s**2) + r*(q**2 + r**2 + s**2)*sin(sqrt(q**2 + r**2 + s**2)))/(q**2 + r**2 + s**2)**(3/2), (-q*(q**2 + r**2 + s**2)*sin(sqrt(q**2 + r**2 + s**2)) + r*s*(1 - cos(sqrt(q**2 + r**2 + s**2)))*sqrt(q**2 + r**2 + s**2))/(q**2 + r**2 + s**2)**(3/2),                                                                             (s**2 + (q**2 + r**2)*cos(sqrt(q**2 + r**2 + s**2)))/(q**2 + r**2 + s**2)]])

'''

