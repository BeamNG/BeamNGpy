import math

class cubic:
    """
    A class for representing explicit cubic polynomials of the form: [ p(x) := a + b*x + c*x^2 + d*x^3 ].
    """

    def __init__(self, a, b, c, d):
        """
        Creates an explicit cubic polynomial

        Args:
            a: The coefficient of the constant term.
            b: The coefficient of the linear term (multiplies x).
            c: The coefficient of the quadratic term (multiplies x^2).
            d: The coefficient of the cubic term (multiplies x^3).
        """
        self.a = a
        self.b = b
        self.c = c
        self.d = d

    def eval(self, x) -> float:
        """
        Evaluates this cubic polynomial, at the given value.

        Args:
            x: The value at which to evaluate this cubic polynomial.

        Returns:
            The evaluation of the cubic polynomial, p(x).
        """
        x_sq = x * x
        return self.a + (x * self.b) + (x_sq * self.c) + (x * x_sq * self.d)

    def eval_upper_only(self, x) -> float:
        """
        Evaluates the upper two terms (quadratic and cubic terms only) of this cubic polynomial, at the given value.

        Args:
            x: The value at which to evaluate this cubic polynomial.

        Returns:
            The evaluation of the upper two terms of this cubic polynomial, p(x).
        """
        x_sq = x * x
        return self.a + (x_sq * self.c) + (x * x_sq * self.d)

    def approx_length(self, x0, x1, n=9000) -> float:
        """
        Computes a numerical approximation of the arc-length of this cubic polynomial.
        The polynomial is approximated as a polyline, and the length of each subsection is summed to a total length.

        Args:
            x0: The start value in x.
            x1: The end value in x.
            n (optional): The number of subdivisions to use across the polynomial

        Returns:
            A numerical approximation of the arc-length of this cubic polynomial.
        """
        div = (x1 - x0) / float(n)
        sum = 0.0
        last = [x0, self.eval(x0)]
        for i in range(n):                                                      # Evaluate the cubic at n points. Store them.
            x = x0 + (i * div)
            y = self.eval(x)
            dx = x - last[0]
            dy = y - last[1]
            sum += math.sqrt((dx * dx) + (dy * dy))
            last = [x, y]
        return sum                                                              # Return the L^2 distance (approximation).
