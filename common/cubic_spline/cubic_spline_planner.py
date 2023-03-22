"""
Cubic spline planner

Author: Atsushi Sakai(@Atsushi_twi)

"""
import math
import numpy as np
import bisect


class Spline:
    """
    Cubic Spline class
    """

    def __init__(self, x, y):
        self.b, self.c, self.d, self.w = [], [], [], []

        self.x = x
        self.y = y

        self.nx = len(x)  # dimension of x
        h = np.diff(x)

        # calc coefficient c
        self.a = [iy for iy in y]

        # calc coefficient c
        A = self.__calc_A(h)
        B = self.__calc_B(h)
        self.c = np.linalg.solve(A, B)
        #  print(self.c1)

        # calc spline coefficient b and d
        for i in range(self.nx - 1):
            self.d.append((self.c[i + 1] - self.c[i]) / (3.0 * h[i]))
            tb = (self.a[i + 1] - self.a[i]) / h[i] - h[i] * \
                (self.c[i + 1] + 2.0 * self.c[i]) / 3.0
            self.b.append(tb)

        self.d.append(self.d[-1])
        self.b.append(self.b[-1])

    def calc(self, t):
        """
        Calc position
        """
        i = self.search_index(t)
        return self.calc_at_i(t, i)

    def calc_at_i(self, t, i):
        dx = t - self.x[i]
        result = self.a[i] + self.b[i] * dx + \
            self.c[i] * dx ** 2.0 + self.d[i] * dx ** 3.0
        return result

    def calcd(self, t):
        """
        Calc first derivative
        """
        i = self.search_index(t)
        return self.calcd_at_i(t, i)

    def calcd_at_i(self, t, i):
        dx = t - self.x[i]
        result = self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx ** 2.0
        return result

    def calcdd(self, t):
        """
        Calc second derivative
        """
        i = self.search_index(t)
        return self.calcdd_at_i(t, i)

    def calcdd_at_i(self, t, i):
        dx = t - self.x[i]
        result = 2.0 * self.c[i] + 6.0 * self.d[i] * dx
        return result

    def calcddd(self, t):
        """
        Calc third derivative
        """
        i = self.search_index(t)
        return self.calcddd_at_i(t, i)

    def calcddd_at_i(self, t, i):
        result = 6.0 * self.d[i]
        return result

    def search_index(self, x):
        """
        search data segment index
        """
        if x < self.x[0]:
            return 0
        if x > self.x[-1]:
            return -1
        return bisect.bisect(self.x, x) - 1

    def __calc_A(self, h):
        """
        calc matrix A for spline coefficient c
        """
        A = np.zeros((self.nx, self.nx))
        A[0, 0] = 1.0
        for i in range(self.nx - 1):
            if i != (self.nx - 2):
                A[i + 1, i + 1] = 2.0 * (h[i] + h[i + 1])
            A[i + 1, i] = h[i]
            A[i, i + 1] = h[i]

        A[0, 1] = 0.0
        A[self.nx - 1, self.nx - 2] = 0.0
        A[self.nx - 1, self.nx - 1] = 1.0
        #  print(A)
        return A

    def __calc_B(self, h):
        """
        calc matrix B for spline coefficient c
        """
        B = np.zeros(self.nx)
        for i in range(self.nx - 2):
            B[i + 1] = 3.0 * (self.a[i + 2] - self.a[i + 1]) / \
                h[i + 1] - 3.0 * (self.a[i + 1] - self.a[i]) / h[i]
        return B


class Spline2D:
    """
    2D Cubic Spline class

    """

    def __init__(self, x, y, ds=None):
        self.s = self.__calc_s(x, y, ds)
        self.sx = Spline(self.s, x)
        self.sy = Spline(self.s, y)

    def __calc_s(self, x, y, ds):
        self.ds = np.hypot(np.diff(x), np.diff(y)) \
            if ds is None else ds
        s = [0]
        s.extend(np.cumsum(self.ds))
        return s

    def calc_position(self, s):
        """
        calc position
        """
        x = self.sx.calc(s)
        y = self.sy.calc(s)
        return x, y

    def calc_curvature(self, s):
        """
        calc curvature
        """
        i = self.sx.search_index(s)
        dx = self.sx.calcd_at_i(s, i)
        ddx = self.sx.calcdd_at_i(s, i)

        # i = self.sy.search_index(s)
        dy = self.sy.calcd_at_i(s, i)
        ddy = self.sy.calcdd_at_i(s, i)

        hypot_2 = dx ** 2 + dy ** 2
        d0 = ddy * dx - ddx * dy
        k = d0 / (hypot_2**(3 / 2))
        return k

    def calc_curvature_d(self, s):
        """
        calc curvature derivative
        """
        i = self.sx.search_index(s)
        dx = self.sx.calcd_at_i(s, i)
        ddx = self.sx.calcdd_at_i(s, i)
        dddx = self.sx.calcddd_at_i(s, i)

        # i = self.sy.search_index(s)
        dy = self.sy.calcd_at_i(s, i)
        ddy = self.sy.calcdd_at_i(s, i)
        dddy = self.sy.calcddd_at_i(s, i)

        hypot_2 = dx ** 2 + dy ** 2
        hypot_2_q = 2 * (dx + dy)
        hypot = math.sqrt(hypot_2)
        d0 = ddy * dx - ddx * dy
        d1 = ddy * ddx + dddy * dx
        d2 = ddx * ddy + dddx * dy
        yaw = math.atan2(dy, dx)
        k = d0 / (hypot**3)
        dk = ( (d1 - d2) * (hypot**3)
              - (3 / 2) * d0 * hypot * hypot_2_q
            ) / (hypot_2**3)
        return yaw, k, dk

    def calc_yaw(self, s):
        """
        calc yaw
        """
        i = self.sx.search_index(s)
        dx = self.sx.calcd_at_i(s, i)

        # i = self.sy.search_index(s)
        dy = self.sy.calcd_at_i(s, i)

        yaw = math.atan2(dy, dx)
        return yaw


def calc_spline_course(x, y, ds=0.1):
    sp = Spline2D(x, y)
    s = list(np.arange(0, sp.s[-1], ds))

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(sp.calc_yaw(i_s))
        rk.append(sp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, s


def main():  # pragma: no cover
    print("Spline 2D test")
    import matplotlib.pyplot as plt
    x = [-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0]
    y = [0.7, -6, 5, 6.5, 0.0, 5.0, -2.0]
    ds = 0.1  # [m] distance of each interpolated points

    sp = Spline2D(x, y)
    s = np.arange(0, sp.s[-1], ds)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(sp.calc_yaw(i_s))
        rk.append(sp.calc_curvature(i_s))

    plt.subplots(1)
    plt.plot(x, y, "xb", label="input")
    plt.plot(rx, ry, "-r", label="spline")
    plt.grid(True)
    plt.axis("equal")
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.legend()

    plt.subplots(1)
    plt.plot(s, [np.rad2deg(iyaw) for iyaw in ryaw], "-r", label="yaw")
    plt.grid(True)
    plt.legend()
    plt.xlabel("line length[m]")
    plt.ylabel("yaw angle[deg]")

    plt.subplots(1)
    plt.plot(s, rk, "-r", label="curvature")
    plt.grid(True)
    plt.legend()
    plt.xlabel("line length[m]")
    plt.ylabel("curvature [1/m]")

    plt.show()


if __name__ == '__main__':
    main()
