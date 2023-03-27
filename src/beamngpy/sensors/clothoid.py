import numpy as np
import math

def FresnelCS(y):
    fn = [0.49999988085884732562, 1.3511177791210715095, 1.3175407836168659241, 1.1861149300293854992, 0.7709627298888346769,
        0.4173874338787963957, 0.19044202705272903923, 0.06655998896627697537, 0.022789258616785717418, 0.0040116689358507943804,
        0.0012192036851249883877]
    fd = [1.0, 2.7022305772400260215, 4.2059268151438492767, 4.5221882840107715516, 3.7240352281630359588, 2.4589286254678152943,
        1.3125491629443702962, 0.5997685720120932908, 0.20907680750378849485, 0.07159621634657901433, 0.012602969513793714191,
        0.0038302423512931250065]
    gn = [0.50000014392706344801, 0.032346434925349128728, 0.17619325157863254363, 0.038606273170706486252, 0.023693692309257725361,
        0.007092018516845033662, 0.0012492123212412087428, 0.00044023040894778468486, -8.80266827476172521e-6, -1.4033554916580018648e-8,
        2.3509221782155474353e-10]
    gd  = [1.0, 2.0646987497019598937, 2.9109311766948031235, 2.6561936751333032911, 2.0195563983177268073, 1.1167891129189363902,
        0.57267874755973172715, 0.19408481169593070798, 0.07634808341431248904, 0.011573247407207865977, 0.0044099273693067311209,
        -0.00009070958410429993314]
    FresnelC = 0.0
    FresnelS = 0.0
    x = abs(y)
    if x < 1.0:
        tt = -((math.pi * 0.5) * x * x)
        t = -(tt * tt)

        # Cosine integral series.
        twofn = 0.0
        fact = 1.0
        denterm = 1.0
        numterm = 1.0
        sum = 1.0
        ratio = 10.0

        while ratio > 1e-34:
            twofn = twofn + 2.0
            fact = fact * twofn * (twofn - 1.0)
            denterm = denterm + 4.0
            numterm = numterm * t
            term = numterm / (fact * denterm)
            sum = sum + term
            ratio = abs(term / sum)
        FresnelC = x * sum

        # Sine integral series.
        twofn = 1.0
        fact = 1.0
        denterm = 3.0
        numterm = 1.0
        sum = 1.0 / 3.0
        ratio = 10.0

        while ratio > 1e-34:
            twofn = twofn + 2.0
            fact = fact * twofn * (twofn - 1.0)
            denterm = denterm + 4.0
            numterm = numterm * t
            term = numterm / (fact * denterm)
            sum = sum + term
            ratio = abs(term / sum)
        FresnelS = (math.pi * 0.5) * sum * x * x * x
    elif x < 6.0:
        # Rational approximation for f.
        sumn = 0.0
        sumd = fd[11]
        for k in range(10):
            sumn = fn[k] + (x * sumn)
            sumd = fd[k] + (x * sumd)
        f = sumn / sumd
        # Rational approximation for g.
        sumn = 0.0
        sumd = gd[11]
        for k in range(10):
            sumn = gn[k] + (x * sumn)
            sumd = gd[k] + (x * sumd)
        g = sumn / sumd
        U = (math.pi * 0.5) * x * x
        SinU = math.sin(U)
        CosU = math.cos(U)
        FresnelC = 0.5 + f * SinU - g * CosU
        FresnelS = 0.5 - f * CosU - g * SinU
    else:
        # x >= 6; asymptotic expansions for f and g.
        tt = -math.pi * x * x
        t = -1.0 / (tt * tt)
        # Expansion for f.
        numterm = -1.0
        term = 1.0
        sum = 1.0
        oldterm = 1.0
        ratio = 10.0
        eps10 = 1e-24
        while ratio > eps10:
            numterm = numterm + 4.0
            term    = term * numterm * (numterm - 2.0) * t
            sum     = sum + term
            absterm = abs(term)
            ratio   = abs(term / sum)
            if oldterm < absterm:
                #print('WARNING - In FresnelCS f not converged to eps')
                ratio = eps10
            oldterm = absterm
        f = sum/(math.pi*x)
        # Expansion for g.
        numterm = -1.0
        term = 1.0
        sum = 1.0
        oldterm = 1.0
        ratio = 10.0
        while ratio > eps10:
            numterm = numterm+ 4.0
            term    = term*numterm*(numterm+2.0)*t
            sum     = sum+term
            absterm = abs(term)
            ratio   = abs(term/sum)
            if oldterm < absterm:
                #print('WARNING - In FresnelCS g not converged to eps')
                ratio = eps10
            oldterm = absterm
        fac = math.pi * x
        g = sum / (fac * fac * x)
        U = (math.pi / 2) * x * x
        SinU = math.sin(U)
        CosU = math.cos(U)
        FresnelC = 0.5 + f * SinU - g * CosU
        FresnelS = 0.5 - f * CosU - g * SinU
    if y < 0:
      FresnelC = -FresnelC
      FresnelS = -FresnelS
    return [FresnelC,FresnelS]

def cross2(A, B):
  return (A[0] * B[1]) - (A[1] * B[0])

def dot2(A, B):
  return  (A[0] * B[0]) + (A[1] * B[1])

def reverse(phi1, phi2, P1, D):
    D_out    = [-D[0], -D[1]]
    P1_out   = [P1[0] + D[0], P1[1] + D[1]]
    return [-phi2, -phi1, P1_out, D_out]

def reflect(phi1, phi2):
    return [-phi1, -phi2]

def rotate(V, alpha):
    W = []
    W.append(V[0] * math.cos(alpha) - V[1] * math.sin(alpha))
    W.append(V[0] * math.sin(alpha) + V[1] * math.cos(alpha))
    return W

def Fresnel2(theta):
    [C,S] = FresnelCS(math.sqrt(2.0 * abs(theta) / math.pi))
    C = C * np.sign(theta)
    S = S * np.sign(theta)
    return [C, S]

def evaluate(theta, phi1, phi2, segno):
    [C1,S1] = Fresnel2(theta)
    [C2,S2] = Fresnel2(theta + phi1 + phi2)
    c  = math.cos(theta + phi1)
    s  = math.sin(theta + phi1)
    f  = math.sqrt(2.0 * math.pi) * (c * (S2 - segno * S1) - s * (C2 - segno * C1))
    df = math.sin(phi2) / max(1e-12, math.sqrt(theta+phi1+phi2)) + segno*math.sin(phi1)/max(1e-12, math.sqrt(theta)) - math.sqrt(2*math.pi)*(s*(S2-segno*S1)+c*(C2-segno*C1))
    return [f, df]

def solve( a, b, phi1, phi2, segno, tol, iterLim ):
    [fa,dfa] = evaluate(a,phi1,phi2,segno)
    [fb,dfb] = evaluate(b,phi1,phi2,segno)
    theta  = (a+b)/2
    [f,df] = evaluate(theta,phi1,phi2,segno)
    err    = b-a
    iter   = 0
    while err > tol and iter < iterLim:
        iter = iter + 1
        NewtonFail = True
        if abs(df) > tol:
            thetaiter = theta - (f / df)
            delta = abs(theta-thetaiter)
            if thetaiter > a and thetaiter < b and delta < 0.5 * err:
                NewtonFail = False
                theta      = thetaiter
                err        = delta
        if NewtonFail:
            if fa * f < 0:
                b  = theta
                fb = f
            else:
                a  = theta
                fa = f
            theta = (a + b) * 0.5
            err   = b-a
        [f,df] = evaluate(theta, phi1, phi2, segno)
    failFlag = iter >= iterLim
    return [theta,iter,failFlag]

def fitEuler(P1,T,d,phi1,phi2,tol,iterLim,reflectFlag):
    iter = 0
    theta = 0
    segno = 1
    t1 = 0
    t2 = math.sqrt( 2*(phi1+phi2)/math.pi )
    [C,S] = FresnelCS(t2)
    h = S*math.cos(phi1)-C*math.sin(phi1)
    if (phi1 > 0) and (h <= 0):
        # C shaped
        if h > tol:
            failFlag = False # solution theta = 0
        else:
            lambd = (1-math.cos(phi1))/(1-math.cos(phi2))
            theta0 = (lambd*lambd)/(1-(lambd*lambd))*(phi1+phi2)
            [theta,iter,failFlag] = solve(0,theta0,phi1,phi2,segno,tol,iterLim)
    else:
        segno = -1
        theta0 = max(0,-phi1)
        theta1 = math.pi/2-phi1
        [theta,iter,failFlag] = solve(theta0,theta1,phi1,phi2,segno,tol,iterLim)

    t1 = segno*math.sqrt(2*theta/math.pi)
    t2 = math.sqrt(2*(theta+phi1+phi2)/math.pi)
    [C1, S1] = FresnelCS(t1)
    [C2, S2] = FresnelCS(t2)
    phi     = phi1 + theta
    a       = d/((S2-S1)*math.sin(phi)+(C2-C1)*math.cos(phi))
    if reflectFlag:
        T0 = rotate(T,phi)
        N0 = rotate(T0,-math.pi/2)
    else:
        T0 = rotate(T,-phi)
        N0 = rotate(T0,math.pi/2)
    m1 = [C1*T0[0], C1*T0[1]]
    m2 = [S1*N0[0], S1*N0[1]]
    m3 = [m1[0] + m2[0], m1[1] + m2[1]]
    m4 = [a * m3[0], a * m3[1]]
    P0 = [P1[0] - m4[0], P1[1] - m4[1]]
    return [P0, T0, N0, a, t1, t2, iter, failFlag]

def completeShape( P1, T1, P2, T2, tol, iterLim ):
    kind = 0
    D    = [P2[0] - P1[0], P2[1] - P1[1]]
    d    = math.sqrt(D[0]*D[0] + D[1]*D[1])
    if d < tol:
        print('WARNING: degenerate case.')
    if tol < 1e-18:
        tol = 1e-18
    phi1 = math.atan2( cross2(T1,D), dot2(T1,D) )
    phi2 = math.atan2( cross2(D,T2), dot2(D,T2) )
    P0   = [0.0, 0.0]
    T0   = [0.0, 0.0]
    N0   = [0.0, 0.0]
    t1   = 0
    t2   = 0
    iter = 0
    failFlag = False
    reverseFlag = False
    if abs(phi1) > abs(phi2):
        [ phi1, phi2, P1, D ] = reverse( phi1, phi2, P1, D )
        reverseFlag = True
    reflectFlag = False
    if phi2 < 0:
        [ phi1, phi2 ] = reflect( phi1, phi2 )
        reflectFlag = True
    a = 0
    if ( (phi1 == 0 ) & (phi2 == math.pi) ) | ( (phi1 == math.pi) & (phi2 == 0 ) ) | ( (phi1 == math.pi) & (phi2 == math.pi) ):
        print('WARNING: ambiguous case: ', phi1, phi2)
    elif (abs(phi1) <= tol) & (abs(phi2) <= tol):
        a = 0
        kind = 2 # straight line
    elif abs(phi1-phi2) <= tol:
        a = phi1
        kind = 1 # circle
    else:
        d_recip = 1.0 / d
        Dnorm = [D[0] * d_recip, D[1] * d_recip]
        [P0, T0, N0, a, t1, t2, iter, failFlag] = fitEuler(P1, Dnorm, d, phi1, phi2, tol, iterLim, reflectFlag)
    return [P0, T0, N0, a, t1, t2, iter, failFlag, reflectFlag, reverseFlag, kind]

def fitClothoid( x0, y0, theta0, x1, y1, theta1, tol ):
    MW_fails = 0
    MW_calls = 0
    MW_calls_deg = 0
    P1 = [x0, y0]
    T1 = [math.cos(theta0),math.sin(theta0)]
    P2 = [x1, y1]
    T2 = [math.cos(theta1),math.sin(theta1)]
    [P0, T0, N0, a, t1, t2, iter, failFlag, reflectFlag, reverseFlag, kind] = completeShape(P1, T1, P2, T2, tol, 100)
    MW_calls += 1
    if failFlag:
        MW_fails += 1
        # warning('non converge')

    if kind == 0:
        dk = math.pi/(a*a)
        L = a*(t2-t1)
        if reverseFlag:
            k = -math.pi*t2/a
        else:
            k = math.pi*t1/a
        if reflectFlag:
            k  = -k
            dk = -dk
    elif kind == 1: # circle
            MW_calls_deg += 1
            dk = 0
            d  = math.sqrt((x1-x0)*(x1-x0)+(y1-y0)*(y1-y0))
            k  = 2*math.sin(a)/d
            L  = a*d/math.sin(a)
            if reverseFlag:
                k = -k
            if reflectFlag:
                k = -k
    else: # straight line
            MW_calls_deg += 1
            dk = 0
            k  = 0
            L  = math.sqrt((x1-x0)*(x1-x0)+(y1-y0)*(y1-y0))
    return [ k, dk, L, iter ]