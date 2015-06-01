import math


class Pacejka94:
    ''' Pacejka 94 lateral tire model.
        http://www.edy.es/dev/docs/pacejka-94-parameters-explained-a-comprehensive-guide/
        http://www.optimumg.com/docs/OptimumTire_Help_File.pdf
    '''

    def __init__(self, coeff):
        # Tire Coefficients
        self.fnomin = coeff['fnomin']
        self.pCy1 = coeff['pcy1']
        self.pDy1 = coeff['pdy1']
        self.pDy2 = coeff['pdy2']
        self.pDy3 = coeff['pdy3']
        self.pEy1 = coeff['pey1']
        self.pEy2 = coeff['pey2']
        self.pEy3 = coeff['pey3']
        self.pEy4 = coeff['pey4']
        self.pKy1 = coeff['pky1']
        self.pKy2 = coeff['pky2']
        self.pKy3 = coeff['pky3']
        self.pHy1 = coeff['phy1']
        self.pHy2 = coeff['phy2']
        self.pHy3 = coeff['phy3']
        self.pVy1 = coeff['pvy1']
        self.pVy2 = coeff['pvy2']
        self.pVy3 = coeff['pvy3']
        self.pVy4 = coeff['pvy4']

    # Calculate Pacejka coefficients, prepend with 'c' for 'coefficient'
    def fy(self, f_z, alpha, gamma):
        '''
        Calculates tire force, F_y
        where f_z    vertical tire load [N]
              alpha  slip angle [rad]
              gamma  camber angle [rad]
        '''
        h = self.cHy(f_z, gamma)
        k = self.cKy(f_z, gamma)

        c = self.pCy1
        d = self.cDy(f_z, gamma)
        e = self.cEy(f_z, alpha, gamma, h)
        v = self.cVy(f_z, gamma)

        b = self.cBy(k, c, d)
        bx1 = self.cBx1(alpha, b, h)

        return d * math.sin(c * math.atan(bx1 - e * (bx1 -
                            math.atan(bx1)))) + v

    def cDy(self, f_z, gamma):
        '''
        Calculates lateral peak factor, D
        where f_z    vertical tire load [N]
              gamma  camber angle [rad]
        '''
        return f_z * (self.pDy1 + self.pDy2 * self.fnorm(f_z)) * (1 - self.pDy3 * gamma * gamma)

    def cKy(self, f_z, gamma):
        '''
        Calculates lateral stiffness, K
        where f_z    vertical tire load [N]
              gamma  camber angle [rad]
        '''
        return self.pKy1 * self.fnomin * math.sin(2 * math.atan(f_z / (self.pKy2
                                                    * self.fnomin))) * (1 - self.pKy3 * math.fabs(gamma))

    def cBy(self, cKy, cCy, cDy):
        '''
        Calculates lateral stiffness factor, B
        where cKy    lateral stiffness, K
              cCy    shape factor, C
              cDy    peak factor, D
        '''
        return cKy/(cCy*cDy)

    def cEy(self, f_z, gamma, alpha, yH):
        '''
        Calculates lateral curvature factor, E
        where f_z    vertical tire load [N]
              gamma  camber angle [rad]
              alpha  slip angle [rad]
              yH     horizontal shift, H
        '''
        sign = math.copysign(1, alpha + yH)
        return (self.pEy1 + self.pEy2 * self.fnorm(f_z)) * (1 - (self.pEy4 * gamma + self.pEy3) * sign)

    def cHy(self, f_z, gamma):
        '''
        Calculates lateral horizontal shift, H
        where f_z    vertical tire load [N]
              gamma  camber angle [rad]
        '''
        return self.pHy1 + self.pHy2 * self.fnorm(f_z) + self.pHy3 * gamma

    def cVy(self, f_z, gamma):
        '''
        Calculates lateral vertical shift, V
        where f_z    vertical tire load [N]
              gamma  camber angle [rad]
        '''
        return (self.pVy1 + self.pVy2 * self.fnorm(f_z) + ((self.pVy3 + self.pVy4 *
                                                            self.fnorm(f_z)) * gamma)) * f_z

    def cBx1(self, alpha, cBy, cHy):
        '''
        Calculates ??? factor, Bx1
        where alpha  slip angle [rad]
              cBy    stiffness factor, B
              cHy    horizontal shift, H
        '''
        return cBy * (alpha + cHy)

    def fnorm(self, f_z):
        '''
        Calculates normalized vertical load, df_z
        where f_z    vertical tire load [N]
        '''
        return (f_z - self.fnomin) / self.fnomin
