from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from typing import TYPE_CHECKING

import numpy as np

from beamngpy import MeshRoad, vec3

if TYPE_CHECKING:
    from beamngpy import Scenario

# User control parameters
GRANULARITY = 100  # The granularity used when evaluating OpenDrive primitives.
DEPTH = 1.0  # The depth (from bottom to top) of the generated mesh roads in BeamNG.


class Road:
    """
    A container for storing a road polyline, before rendering in BeamNG.
    """

    def __init__(self, name, nodes):
        self.name = name
        self.nodes = nodes


class ExpCubic:
    """
    A container for storing explicit polynomials (used for elevation profiles, width profiles, and lane offset profiles).
    """

    def __init__(self, s, a, b, c, d):
        self.s = float(s)
        self.a, self.b, self.c, self.d = float(a), float(b), float(c), float(d)


class LineSegment:
    """
    A container for storing line segments.
    """

    def __init__(
        self, id, s, x, y, hdg, length, elev=None, width=None, lane_offset=None
    ):
        self.id = int(id)
        self.s = float(s)
        self.x, self.y = float(x), float(y)
        self.hdg = float(hdg)
        self.length = float(length)
        self.elev, self.width, self.lane_offset = elev, width, lane_offset

    def discretize(self):
        eval_range = GRANULARITY + 1
        granularity_inv = 1.0 / GRANULARITY
        # The geodesic length of the road (straight line from start to end positions).
        length = self.length
        # The heading angle at the start position, in radians.
        hdg = self.hdg
        # The reference line unit vector, s.
        s = vec3(math.cos(hdg), math.sin(hdg)).normalize()
        # The road start position.
        start = vec3(self.x, self.y)
        elev_profiles, width_profiles, lo = self.elev, self.width, self.lane_offset
        nodes = []
        # Will evaluate each parametric cubic, discretized at the chosen granularity.
        for i in range(eval_range):
            # The parameter q, in [0, geodesic_length], mapped from p.
            q = i * granularity_inv * length
            # The linearly-interpolated point, in world space (x, y).
            world = start + (s * q)
            dd = (world - start).length()
            # Get the appropriate elevation polynomial for the current s value.
            ep = OpenDriveImporter.get_elevation_profile(self.s + q, elev_profiles)
            ds = self.s - ep.s + q
            ds2 = ds * ds
            ds3 = ds2 * ds
            # Compute the elevation value (scalar). This is in [0, geodesic_length] range.
            elev = ep.a + (ds * ep.b) + (ds2 * ep.c) + (ds3 * ep.d)
            # Compute the sum of the widths at this point. This is in [0, geodesic_length] range.
            width, signed_offset = OpenDriveImporter.compute_width_sum(
                self.s, q, width_profiles, lo
            )
            nodes.append([world.x, world.y, elev, width, DEPTH, signed_offset])
        return nodes


class Arc:
    """
    A container for storing circlular arcs (constant curvature).
    """

    def __init__(
        self,
        id,
        s,
        x,
        y,
        hdg,
        length,
        curvature,
        elev=None,
        width=None,
        lane_offset=None,
    ):
        self.id = int(id)
        self.s = float(s)
        self.x, self.y = float(x), float(y)
        self.hdg = float(hdg)
        self.length = float(length)
        self.curvature = float(curvature)
        self.elev, self.width, self.lane_offset = elev, width, lane_offset

    def discretize(self):
        eval_range = GRANULARITY + 1
        granularity_inv = 1.0 / GRANULARITY
        # The geodesic length of the road (straight line from start to end positions).
        length = self.length
        # The heading angle at the start position, in radians.
        hdg = self.hdg
        # The road start position.
        start = vec3(self.x, self.y)
        # The curvature (kappa) value. This is constant for an arc.
        curvature = self.curvature
        elev_profiles, width_profiles, lo = self.elev, self.width, self.lane_offset
        nodes = []
        for i in range(eval_range):
            # The parameter q, in [0, geodesic_length], mapped from p.
            q = i * granularity_inv * length
            # Evaluate the arc at parameter q, to get the 2D world space position.
            x, y, tang, k = OpenDriveImporter.evalClothoid(
                start.x, start.y, hdg, curvature, 0.0, q
            )
            # Get the appropriate elevation polynomial for the current s value.
            ep = OpenDriveImporter.get_elevation_profile(self.s + q, elev_profiles)
            ds = self.s - ep.s + q
            ds2 = ds * ds
            ds3 = ds2 * ds
            # Compute the elevation value (scalar). This is in [0, geodesic_length] range.
            elev = ep.a + (ds * ep.b) + (ds2 * ep.c) + (ds3 * ep.d)
            # Compute the sum of the widths at this point. This is in [0, geodesic_length] range.
            width, signed_offset = OpenDriveImporter.compute_width_sum(
                self.s, q, width_profiles, lo
            )
            nodes.append([x, y, elev, width, DEPTH, signed_offset])
        return nodes


class Spiral:
    """
    A class for representing and processing clothoid spirals (linear curvature).
    """

    def __init__(
        self,
        id,
        s,
        x,
        y,
        hdg,
        length,
        start_k,
        end_k,
        elev=None,
        width=None,
        lane_offset=None,
    ):
        self.id = int(id)
        self.s = float(s)
        self.x, self.y = float(x), float(y)
        self.hdg = float(hdg)
        self.length = float(length)
        self.start_k, self.end_k = float(start_k), float(end_k)
        self.elev, self.width, self.lane_offset = elev, width, lane_offset

    def discretize(self):
        eval_range = GRANULARITY + 1
        granularity_inv = 1.0 / GRANULARITY
        # The geodesic length of the road (straight line from start to end positions).
        length = self.length
        # The heading angle at the start position, in radians.
        hdg = self.hdg
        # The road start position.
        start = vec3(self.x, self.y)
        # The start curvature (kappa) value. kappa changes linearly for a clothoid.
        start_k = self.start_k
        # The first derivative of the curvature, wrt arc length.
        curv_slope = (self.end_k - start_k) / length
        elev_profiles, width_profiles, lo = self.elev, self.width, self.lane_offset
        nodes = []
        for i in range(eval_range):
            # The parameter q, in [0, geodesic_length], mapped from p.
            q = i * granularity_inv * length
            # Evaluate the clothoid spiral at parameter q, to get the 2D world space position.
            x, y, tang, k = OpenDriveImporter.evalClothoid(
                start.x, start.y, hdg, start_k, curv_slope, q
            )
            # Get the appropriate elevation polynomial for the current s value.
            ep = OpenDriveImporter.get_elevation_profile(self.s + q, elev_profiles)
            ds = self.s - ep.s + q
            ds2 = ds * ds
            ds3 = ds2 * ds
            # Compute the elevation value (scalar). This is in [0, geodesic_length] range.
            elev = ep.a + (ds * ep.b) + (ds2 * ep.c) + (ds3 * ep.d)
            # Compute the sum of the widths at this point. This is in [0, geodesic_length] range.
            width, signed_offset = OpenDriveImporter.compute_width_sum(
                self.s, q, width_profiles, lo
            )
            nodes.append([x, y, elev, width, DEPTH, signed_offset])
        return nodes


class Poly3:
    """
    A container for storing explicit cubic polynomials.
    """

    def __init__(
        self,
        id,
        s,
        x,
        y,
        hdg,
        length,
        a,
        b,
        c,
        d,
        elev=None,
        width=None,
        lane_offset=None,
    ):
        self.id = int(id)
        self.s = float(s)
        self.x, self.y = float(x), float(y)
        self.hdg = float(hdg)
        self.length = float(length)
        self.a, self.b, self.c, self.d = float(a), float(b), float(c), float(d)
        self.elev, self.width, self.lane_offset = elev, width, lane_offset

    def discretize(self):
        eval_range = GRANULARITY + 1
        granularity_inv = 1.0 / GRANULARITY
        # The geodesic length of the road (straight line from start to end positions).
        length = self.length
        # The heading angle at the start position, in radians.
        hdg = self.hdg
        cos_hdg, sin_hdg = math.cos(hdg), math.sin(hdg)
        # The reference line space (s, t) axes.
        s, t = vec3(cos_hdg, sin_hdg).normalize(), vec3(-sin_hdg, cos_hdg).normalize()
        # The road start position.
        start = vec3(self.x, self.y)
        # The individual coefficients.
        cA, cB, cC, cD = self.a, self.b, self.c, self.d
        elev_profiles, width_profiles, lo = self.elev, self.width, self.lane_offset
        nodes = []
        # Will evaluate each parametric cubic, discretized at the chosen granularity.
        for i in range(eval_range):
            # The parameter p, in [0, 1], and its powers.
            p = i * granularity_inv
            # The parameter q, in [0, geodesic_length], mapped from p.
            q = p * length
            q2 = q * q
            q3 = q2 * q
            # v(q) (the lateral deviation from the reference line).
            v = cA + (q * cB) + (q2 * cC) + (q3 * cD)
            # Project from (s, t) space to world space (x, y).
            world = start + (s * q) + (t * v)
            # Get the appropriate elevation polynomial for the current s value.
            ep = OpenDriveImporter.get_elevation_profile(self.s + q, elev_profiles)
            ds = self.s - ep.s + q
            ds2 = ds * ds
            ds3 = ds2 * ds
            # Compute the elevation value (scalar). This is in [0, geodesic_length] range.
            elev = ep.a + (ds * ep.b) + (ds2 * ep.c) + (ds3 * ep.d)
            # Compute the sum of the widths at this point. This is in [0, geodesic_length] range.
            width, signed_offset = OpenDriveImporter.compute_width_sum(
                self.s, q, width_profiles, lo
            )
            nodes.append([world.x, world.y, elev, width, DEPTH, signed_offset])
        return nodes


class ParamPoly3:
    """
    A container for storing parametric cubic polynomials.
    """

    def __init__(
        self,
        id,
        s,
        x,
        y,
        hdg,
        length,
        aU,
        bU,
        cU,
        dU,
        aV,
        bV,
        cV,
        dV,
        pRange,
        elev=None,
        width=None,
        lane_offset=None,
    ):
        self.id = int(id)
        self.s = float(s)
        self.x, self.y = float(x), float(y)
        self.hdg = float(hdg)
        self.length = float(length)
        self.aU, self.bU, self.cU, self.dU = float(aU), float(bU), float(cU), float(dU)
        self.aV, self.bV, self.cV, self.dV = float(aV), float(bV), float(cV), float(dV)
        self.pRange = str(pRange)
        self.elev, self.width, self.lane_offset = elev, width, lane_offset

    def discretize(self):
        eval_range = GRANULARITY + 1
        granularity_inv = 1.0 / GRANULARITY
        # The geodesic length of the road (straight line from start to end positions).
        length = self.length
        # The heading angle at the start position, in radians.
        hdg = self.hdg
        cos_hdg, sin_hdg = math.cos(hdg), math.sin(hdg)
        # The reference line space (s, t) axes.
        s, t = vec3(cos_hdg, sin_hdg).normalize(), vec3(-sin_hdg, cos_hdg).normalize()
        # The road start position.
        start = vec3(self.x, self.y)
        # The individual coefficients of the u(p) equation.
        aU, bU, cU, dU = self.aU, self.bU, self.cU, self.dU
        # The individual coefficients of the v(p) equation.
        aV, bV, cV, dV = self.aV, self.bV, self.cV, self.dV
        # A flag which indicates if the parameters are in [0, geodesic_length] or [0, 1].
        is_arc_length = self.pRange == "arcLength"
        elev_profiles, width_profiles, lo = self.elev, self.width, self.lane_offset
        nodes = []
        # Will evaluate each parametric cubic, discretized at the chosen granularity.
        for i in range(eval_range):
            # The parameter p, in [0, 1], and its powers.
            p = i * granularity_inv
            # The parameter q, in [0, geodesic_length], mapped from p.
            q = p * length
            if is_arc_length:
                q2 = q * q
                q3 = q2 * q
                u = aU + (q * bU) + (q2 * cU) + (q3 * dU)  # u(q), v(q).
                v = aV + (q * bV) + (q2 * cV) + (q3 * dV)
            else:
                p2 = p * p
                p3 = p2 * p
                u = aU + (p * bU) + (p2 * cU) + (p3 * dU)  # u(p), v(p).
                v = aV + (p * bV) + (p2 * cV) + (p3 * dV)
            # Project from (s, t) space to world space (x, y).
            world = start + (s * u) + (t * v)
            # Get the appropriate elevation polynomial for the current s value.
            ep = OpenDriveImporter.get_elevation_profile(self.s + q, elev_profiles)
            ds = self.s - ep.s + q
            ds2 = ds * ds
            ds3 = ds2 * ds
            # Compute the elevation value (scalar). This is in [0, geodesic_length] range.
            elev = ep.a + (ds * ep.b) + (ds2 * ep.c) + (ds3 * ep.d)
            # Compute the sum of the widths at this point. This is in [0, geodesic_length] range.
            width, signed_offset = OpenDriveImporter.compute_width_sum(
                self.s, q, width_profiles, lo
            )
            nodes.append([world.x, world.y, elev, width, DEPTH, signed_offset])
        return nodes


class OpenDriveImporter:

    @staticmethod
    def FresnelCS(y):
        fn = [
            0.49999988085884732562,
            1.3511177791210715095,
            1.3175407836168659241,
            1.1861149300293854992,
            0.7709627298888346769,
            0.4173874338787963957,
            0.19044202705272903923,
            0.06655998896627697537,
            0.022789258616785717418,
            0.0040116689358507943804,
            0.0012192036851249883877,
        ]
        fd = [
            1.0,
            2.7022305772400260215,
            4.2059268151438492767,
            4.5221882840107715516,
            3.7240352281630359588,
            2.4589286254678152943,
            1.3125491629443702962,
            0.5997685720120932908,
            0.20907680750378849485,
            0.07159621634657901433,
            0.012602969513793714191,
            0.0038302423512931250065,
        ]
        gn = [
            0.50000014392706344801,
            0.032346434925349128728,
            0.17619325157863254363,
            0.038606273170706486252,
            0.023693692309257725361,
            0.007092018516845033662,
            0.0012492123212412087428,
            0.00044023040894778468486,
            -8.80266827476172521e-6,
            -1.4033554916580018648e-8,
            2.3509221782155474353e-10,
        ]
        gd = [
            1.0,
            2.0646987497019598937,
            2.9109311766948031235,
            2.6561936751333032911,
            2.0195563983177268073,
            1.1167891129189363902,
            0.57267874755973172715,
            0.19408481169593070798,
            0.07634808341431248904,
            0.011573247407207865977,
            0.0044099273693067311209,
            -0.00009070958410429993314,
        ]
        FresnelC, FresnelS = None, None
        eps = 1e-7

        x = abs(y)
        if x < 1.0:
            f1 = (math.pi / 2) * x * x
            t = -f1 * f1
            # Cosine integral series
            twofn = 0.0
            fact = 1.0
            denterm = 1.0
            numterm = 1.0
            sum = 1.0
            ratio = 10.0

            while ratio > eps:
                twofn = twofn + 2.0
                fact = fact * twofn * (twofn - 1.0)
                denterm = denterm + 4.0
                numterm = numterm * t
                term = numterm / (fact * denterm)
                sum = sum + term
                ratio = abs(term / sum)

            FresnelC = x * sum

            # Sine integral series
            twofn = 1.0
            fact = 1.0
            denterm = 3.0
            numterm = 1.0
            sum = 1.0 / 3.0
            ratio = 10.0

            while ratio > eps:
                twofn = twofn + 2.0
                fact = fact * twofn * (twofn - 1.0)
                denterm = denterm + 4.0
                numterm = numterm * t
                term = numterm / (fact * denterm)
                sum = sum + term
                ratio = abs(term / sum)

            FresnelS = (math.pi / 2) * sum * x * x * x

        elif x < 6.0:
            # Rational approximation for f
            sumn = 0.0
            sumd = fd[11]
            for k in range(10, -1, -1):
                sumn = fn[k] + x * sumn
                sumd = fd[k] + x * sumd

            f = sumn / sumd
            # Rational approximation for  g
            sumn = 0.0
            sumd = gd[11]
            for k in range(10, -1, -1):
                sumn = gn[k] + x * sumn
                sumd = gd[k] + x * sumd
            g = sumn / sumd
            U = (math.pi / 2) * x * x
            SinU = math.sin(U)
            CosU = math.cos(U)
            FresnelC = 0.5 + f * SinU - g * CosU
            FresnelS = 0.5 - f * CosU - g * SinU
        else:
            # x >= 6; asymptotic expansions for  f  and  g
            t = -math.pow((math.pi * x * x), -2)
            # Expansion for  f
            numterm = -1.0
            term = 1.0
            sum = 1.0
            oldterm = 1.0
            ratio = 10.0
            eps10 = 0.1 * eps
            while ratio > eps10:
                numterm = numterm + 4.0
                term = term * numterm * (numterm - 2.0) * t
                sum = sum + term
                absterm = abs(term)
                ratio = abs(term / sum)
                if oldterm < absterm:
                    print("WARNING: FresnelCS f not converged to eps.")
                    ratio = eps10
                oldterm = absterm
            f = sum / (math.pi * x)
            # Expansion for  g
            numterm = -1.0
            term = 1.0
            sum = 1.0
            oldterm = 1.0
            ratio = 10.0
            eps10 = 0.1 * eps
            while ratio > eps10:
                numterm = numterm + 4.0
                term = term * numterm * (numterm + 2.0) * t
                sum = sum + term
                absterm = abs(term)
                ratio = abs(term / sum)
                if oldterm < absterm:
                    print("WARNING: FresnelCS g not converged to eps.")
                    ratio = eps10
                oldterm = absterm
            g = sum / ((math.pi * x) * (math.pi * x) * x)
            U = (math.pi / 2) * x * x
            SinU = math.sin(U)
            CosU = math.cos(U)
            FresnelC = 0.5 + f * SinU - g * CosU
            FresnelS = 0.5 - f * CosU - g * SinU
        if y < 0:
            FresnelC = -FresnelC
            FresnelS = -FresnelS

        return FresnelC, FresnelS

    @staticmethod
    def rLommel(mu, nu, b):
        tmp = 1.0 / ((mu + nu + 1.0) * (mu - nu + 1.0))
        res = tmp
        for n in range(1, 101):
            tmp = tmp * (-b / (2 * n + mu - nu + 1)) * (b / (2 * n + mu + nu + 1))
            res = res + tmp
            if abs(tmp) < abs(res) * 1e-50:
                break
        return res

    @staticmethod
    def evalXYazero(b):
        X = np.zeros(16)
        Y = np.zeros(16)
        sb = math.sin(b)
        cb = math.cos(b)
        b2 = b * b
        if abs(b) < 1e-3:
            X[0] = 1 - (b2 / 6) * (1 - (b2 / 20) * (1 - (b2 / 42)))
            Y[0] = (b / 2) * (1 - (b2 / 12) * (1 - (b2 / 30)))
        else:
            X[0] = sb / b
            Y[0] = (1 - cb) / b
        # use recurrence in the stable part.
        m = min([max([1, np.floor(2 * b)]), 15])
        for k in range(int(m) - 1):
            X[k + 1] = (sb - k * Y[k]) / b
            Y[k + 1] = (k * X[k] - cb) / b
        # use Lommel for the unstable part.
        if m < 15:
            A = b * sb
            D = sb - b * cb
            B = b * D
            C = -b2 * sb
            rLa = OpenDriveImporter.rLommel(m + 1 / 2, 3 / 2, b)
            rLd = OpenDriveImporter.rLommel(m + 1 / 2, 1 / 2, b)
            for k in range(int(m), 15):
                rLb = OpenDriveImporter.rLommel(k + 3 / 2, 1 / 2, b)
                rLc = OpenDriveImporter.rLommel(k + 3 / 2, 3 / 2, b)
                X[k + 1] = (k * A * rLa + B * rLb + cb) / (1 + k)
                Y[k + 1] = (C * rLc + sb) / (2 + k) + D * rLd
                rLa = rLc
                rLd = rLb
        return X, Y

    @staticmethod
    def evalXYaSmall(a, b):
        [X0, Y0] = OpenDriveImporter.evalXYazero(b)
        tmpX = np.zeros(5)
        tmpY = np.zeros(5)

        tmpX[0] = X0[0] - (a / 2) * Y0[1]
        tmpY[0] = Y0[0] + (a / 2) * X0[1]
        t = 1
        aa = -(a / 2) * (a / 2)
        for n in range(1, 4):
            ii = 4 * n
            t = t * (aa / (2 * n * (2 * n - 1)))
            bf = a / (4 * n + 2)
            tmpX[n] = t * (X0[ii] - bf * Y0[ii + 2])
            tmpY[n] = t * (Y0[ii] + bf * X0[ii + 2])
        X = sum(tmpX)
        Y = sum(tmpY)
        return X, Y

    @staticmethod
    def evalXYaLarge(a, b):
        s = np.sign(a)
        z = math.sqrt(abs(a) / math.pi)
        ell = s * b / math.sqrt(abs(a) * math.pi)
        g = -0.5 * s * b * b / abs(a)
        Cl, Sl = OpenDriveImporter.FresnelCS(ell)
        Cz, Sz = OpenDriveImporter.FresnelCS(ell + z)
        dC = Cz - Cl
        dS = Sz - Sl
        cg = math.cos(g) / z
        sg = math.sin(g) / z
        X = cg * dC - s * sg * dS
        Y = sg * dC + s * cg * dS
        return X, Y

    @staticmethod
    def GeneralizedFresnelCS(a, b, c):
        epsi = 1e-2  # best threshold.
        if abs(a) < epsi:  # case: 'a' small.
            [X, Y] = OpenDriveImporter.evalXYaSmall(a, b)
        else:
            [X, Y] = OpenDriveImporter.evalXYaLarge(a, b)
        cc = math.cos(c)
        ss = math.sin(c)
        xx = X
        yy = Y
        X = xx * cc - yy * ss
        Y = xx * ss + yy * cc
        return X, Y

    @staticmethod
    def evalClothoid(x0, y0, theta0, kappa, dkappa, s):
        [C, S] = OpenDriveImporter.GeneralizedFresnelCS(
            dkappa * s * s, kappa * s, theta0
        )
        X = x0 + s * C
        Y = y0 + s * S
        th = theta0 + s * (kappa + s * (dkappa / 2))
        k = kappa + s * dkappa
        return X, Y, th, k

    # From a collection of elevation profiles, find the appropriate one.  This is the profile who's s value is closest and below the given s value.
    @staticmethod
    def get_elevation_profile(s, profiles):
        num_profiles = len(profiles)
        if num_profiles == 0:
            print("!!! WARNING:  no elevation profile found at ", s)
            # If there are no elevation profiles for the current geometry, return a constant default.
            return ExpCubic(0.0, 0.0, 0.0, 0.0, 0.0)

        profile_id = 0
        closest_so_far = 1e24
        # Find the profile from the list which is closest below the given s value. This is the relevant profile.
        for i in range(num_profiles):
            d = s - profiles[i].s
            if d >= 0.0 and d < closest_so_far:
                profile_id, closest_so_far = i, d
        return profiles[profile_id]

    # Compute the sum of the lane widths (left and right together) at some parameter position.
    @staticmethod
    def compute_width_sum(s, q, width_data, lane_offset):
        lane_groups = {}
        profile_s = 99999999999999999
        closest_so_far = 1e24
        for (
            k
        ) in width_data.keys():  # From the width data, find the appropriate lane group.
            d = s - k + q
            if d >= 0.0 and d < closest_so_far:
                lane_groups = width_data[k]
                profile_s = k
                closest_so_far = d

        sum, left_sum, right_sum = 0.0, 0.0, 0.0
        # Sum over all the relevant profiles (one for each lane) to get the final summed width of the entire road, at q.
        for k, wp_list in lane_groups.items():
            # Find the profile from the list which is closest below the given s value. This is the relevant profile.
            profile_id = 999999999999
            closest_so_far = 1e24
            for i in range(len(wp_list)):
                d = s + q - profile_s - wp_list[i].s
                if d >= 0.0 and d < closest_so_far:
                    profile_id, closest_so_far = i, d
            wp = wp_list[profile_id]
            ds = s - profile_s + q - wp_list[profile_id].s
            ds2 = ds * ds
            ds3 = ds2 * ds
            lane_width = wp.a + (ds * wp.b) + (ds2 * wp.c) + (ds3 * wp.d)
            sum = sum + lane_width
            if k < 0:  # Sum also the left and right sides of the road, separately.
                left_sum = left_sum - lane_width
            else:
                right_sum = right_sum + lane_width

        # Compute the encoded lane offset from the .xodr file. We add this to our own computed offset.
        encoded_lo = 0.0
        if len(lane_offset) > 0:
            profile_id = 0.0
            closest_so_far = 1e24
            for i in range(len(lane_offset)):
                d = s + q - lane_offset[i].s
                if d >= 0.0 and d < closest_so_far:
                    profile_id, closest_so_far = i, d
            lo = lane_offset[profile_id]
            ds = s - lo.s + q
            ds2 = ds * ds
            ds3 = ds2 * ds
            encoded_lo = lo.a + (ds * lo.b) + (ds2 * lo.c) + (ds3 * lo.d)

        # The midpoint of interval [left_sum, right_sum] is the offset by which the reference line needs to shift laterally.
        signed_offset = -encoded_lo - (left_sum + right_sum) * 0.5

        return sum, signed_offset

    # Combines all the geometry data into the primitive instances.  Elevation profiles, width profiles, etc.
    @staticmethod
    def combine_geometry_data(
        lines, arcs, spirals, polys, cubics, elevations, widths, lane_offsets
    ):
        for prim in lines + arcs + spirals + polys + cubics:
            prim.elev = elevations
            prim.width = widths
            prim.lane_offset = lane_offsets
        return lines, arcs, spirals, polys, cubics

    # Extracts the road data, per OpenDrive primitive.
    @staticmethod
    def extract_road_data(filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        final_lines, final_arcs, final_spirals, final_polys, final_cubics = (
            [],
            [],
            [],
            [],
            [],
        )
        for child in root:  # Traverse through the xml tree, from its head.
            if child.tag == "road":
                lines, arcs, spirals, polys, cubics = [], [], [], [], []
                elevations = []
                widths_data = {}
                lane_offsets = []
                id = child.attrib["id"]  # The unique road id number.
                for i in child:

                    # Take all elevation information.
                    # Store any given elevation profiles for this 'road'.  Note: there may be multiple, with different s values.
                    if i.tag == "elevationProfile":
                        for j in i:
                            if j.tag == "elevation":
                                elevations.append(
                                    ExpCubic(
                                        j.attrib["s"],
                                        j.attrib["a"],
                                        j.attrib["b"],
                                        j.attrib["c"],
                                        j.attrib["d"],
                                    )
                                )

                    # Take all width information.
                    # Store any given road width profiles for this 'road'.  Note: we will sum up to get total widths.
                    elif i.tag == "lanes":
                        for j in i:
                            # There can be multiple laneSection tages in a road, eg all with different s values.
                            if j.tag == "laneSection":
                                laneSection_s = float(j.attrib["s"])
                                widths = {}
                                for k in j:
                                    if k.tag == "left" or k.tag == "right":
                                        for l in k:
                                            if l.tag == "lane":
                                                # We also store the lane Id of each road, since each lane can have multiple width profiles.
                                                lane_id = int(l.attrib["id"])
                                                # There can be multiple width definitions per lane.
                                                width_per_lane = []
                                                for m in l:
                                                    if m.tag == "width":
                                                        lane_id = int(lane_id)
                                                        width_per_lane.append(
                                                            ExpCubic(
                                                                m.attrib["sOffset"],
                                                                m.attrib["a"],
                                                                m.attrib["b"],
                                                                m.attrib["c"],
                                                                m.attrib["d"],
                                                            )
                                                        )
                                                widths[lane_id] = width_per_lane
                                # Store the widths by lane section s value, as well as by lane id.
                                widths_data[laneSection_s] = widths
                            elif j.tag == "laneOffset":
                                lane_offsets.append(
                                    ExpCubic(
                                        j.attrib["s"],
                                        j.attrib["a"],
                                        j.attrib["b"],
                                        j.attrib["c"],
                                        j.attrib["d"],
                                    )
                                )

                    # Take all geometry information.
                    elif i.tag == "planView":
                        for j in i:
                            # Iterate over all the geometry elements in the planView. Note: there may be multiple, with different s values.
                            if j.tag == "geometry":
                                # The start position, in reference line space.
                                s = j.attrib["s"]
                                # The x, y coordinates of the starting position, in world space.
                                x, y = j.attrib["x"], j.attrib["y"]
                                # The heading angle at the start, in radians.
                                hdg = j.attrib["hdg"]
                                # The geodesic length of the curve (straight line from start to end), in world space.
                                length = j.attrib["length"]
                                for k in j:
                                    if k.tag == "line":
                                        lines.append(
                                            LineSegment(id, s, x, y, hdg, length)
                                        )
                                    elif k.tag == "arc":
                                        arcs.append(
                                            Arc(
                                                id,
                                                s,
                                                x,
                                                y,
                                                hdg,
                                                length,
                                                k.attrib["curvature"],
                                            )
                                        )
                                    elif k.tag == "spiral":
                                        spirals.append(
                                            Spiral(
                                                id,
                                                s,
                                                x,
                                                y,
                                                hdg,
                                                length,
                                                k.attrib["curvStart"],
                                                k.attrib["curvEnd"],
                                            )
                                        )
                                    elif k.tag == "poly3":
                                        polys.append(
                                            Poly3(
                                                id,
                                                s,
                                                x,
                                                y,
                                                hdg,
                                                length,
                                                k.attrib["a"],
                                                k.attrib["b"],
                                                k.attrib["c"],
                                                k.attrib["d"],
                                            )
                                        )
                                    elif k.tag == "paramPoly3":
                                        pRange = "normalized"
                                        if "pRange" in k.attrib:
                                            pRange = k.attrib["pRange"]
                                        cubics.append(
                                            ParamPoly3(
                                                id,
                                                s,
                                                x,
                                                y,
                                                hdg,
                                                length,
                                                k.attrib["aU"],
                                                k.attrib["bU"],
                                                k.attrib["cU"],
                                                k.attrib["dU"],
                                                k.attrib["aV"],
                                                k.attrib["bV"],
                                                k.attrib["cV"],
                                                k.attrib["dV"],
                                                pRange,
                                            )
                                        )

                # Combine all the data which was collected for this road into single structures based on the primitive type.
                if (
                    len(lines) == 0
                    and len(arcs) == 0
                    and len(spirals) == 0
                    and len(polys) == 0
                    and len(cubics) == 0
                ):
                    continue
                lines, arcs, spirals, polys, cubics = (
                    OpenDriveImporter.combine_geometry_data(
                        lines,
                        arcs,
                        spirals,
                        polys,
                        cubics,
                        elevations,
                        widths_data,
                        lane_offsets,
                    )
                )
                final_lines = final_lines + lines
                final_arcs = final_arcs + arcs
                final_spirals = final_spirals + spirals
                final_polys = final_polys + polys
                final_cubics = final_cubics + cubics
        return final_lines, final_arcs, final_spirals, final_polys, final_cubics

    # Adds the appropriate lateral offset to each road polyline.  This is needed because the road reference line may not be in the center of the road - we need it to be, so we adjust.
    @staticmethod
    def add_lateral_offset(roads):
        offset_roads = []
        for r in roads:
            offset_nodes = []
            n1, n2 = [], []
            for i in range(len(r.nodes)):
                n = r.nodes[i]
                p_old = vec3(n[0], n[1])
                if i == 0:
                    n1, n2 = r.nodes[i], r.nodes[i + 1]
                elif i == len(r.nodes) - 1:
                    n1, n2 = r.nodes[i - 1], r.nodes[i]
                else:
                    n1, n2 = r.nodes[i - 1], r.nodes[i + 1]
                p1, p2 = vec3(n1[0], n1[1]), vec3(n2[0], n2[1])
                s = (p2 - p1).normalize()
                t = vec3(s.y, -s.x)
                p_new = p_old + (t * n[5])
                offset_nodes.append([p_new.x, p_new.y, n[2], n[3], n[4]])
            offset_roads.append(Road(r.name, offset_nodes))
        return offset_roads

    # Adjusts the global road network elevation, to make it appear at the right height in beamng.  Uses a given offset.
    @staticmethod
    def adjust_elevation(roads, min_elev=5.0):
        # Compute the lowest elevation value which exists in the whole imported data.
        lowest_elev = 1e99
        for r in roads:
            for i in range(len(r.nodes)):
                lowest_elev = min(lowest_elev, r.nodes[i][2])
        # The vertical offset, by which to adjust all polylines.
        dz = min_elev - lowest_elev
        for r in roads:  # Apply the vertical offset to all road network polylines.
            for i in range(len(r.nodes)):
                r.nodes[i][2] = r.nodes[i][2] + dz
        return roads

    @staticmethod
    def import_xodr(filename, scenario: Scenario):

        # Extract the road data primitives from the OpenDrive file.
        print("Extracting road data from file...")
        lines, arcs, spirals, polys, cubics = OpenDriveImporter.extract_road_data(
            filename
        )
        print(
            "Primitives to import:  lines:",
            len(lines),
            "; arcs:",
            len(arcs),
            "; spirals:",
            len(spirals),
            "; explicit cubics:",
            len(polys),
            "; parametric cubics:",
            len(cubics),
        )

        # Generate separate R^3 road polylines from each imported OpenDrive primitive data.
        print("Generating geometric primitives...")
        roads = []
        for prim in lines + arcs + spirals + polys + cubics:
            roads.append(Road("imported_" + str(prim.id), prim.discretize()))

        # Perform offset re-computation to get the correct road reference line for BeamNG.
        print("Adding road lateral offsetting...")
        roads = OpenDriveImporter.add_lateral_offset(roads)

        # Adjust the elevation of the road polylines so they can be rendered appropriately in the BeamNG world.
        print("Adjusting global elevation...")
        roads = OpenDriveImporter.adjust_elevation(roads)

        # Create the all the roads from the road polyline data (which came from various OpenDrive primitive evaluators).
        print("Loading import in scenario...")
        for r in roads:
            mesh_road = MeshRoad(r.name)
            mesh_road.add_nodes(*r.nodes)
            scenario.add_mesh_road(mesh_road)

        print("Import complete.")
