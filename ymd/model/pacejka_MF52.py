import math


class PacejkaMF52:
    ''' Pacejka MF5.2 tire model.
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
        self.rCy1 = coeff['rcy1']
        self.rBy1 = coeff['rby1']
        self.rBy2 = coeff['rby2']
        self.rBy3 = coeff['rby3']
        self.rHy1 = coeff['rhy1']
        self.rHy2 = coeff['rhy2']
        self.rVy1 = coeff['rvy1']
        self.rVy2 = coeff['rvy2']
        self.rVy3 = coeff['rvy3']
        self.rVy4 = coeff['rvy4']
        self.rVy5 = coeff['rvy5']
        self.rVy6 = coeff['rvy6']
        self.rEy1 = coeff['rey1']
        self.rEy2 = coeff['rey2']

    def fnorm(self, f_z):
        '''
        Calculates normalized vertical load, df_z
        where f_z    vertical tire load [N]
        '''
        return (f_z - self.fnomin) / self.fnomin

    def calc_fy(self, f_z, alpha, kappa, gamma):
        '''
        Calculates tire force, F_y
        where f_z    vertical tire load [N]
              alpha  slip angle [rad]
              gamma  camber angle [rad]
              kappa  slip ratio
        '''

        # Shift factors
        cSHy = self.pHy1 + self.pHy2 * self.fnorm(f_z) + self.pHy3 * gamma
        cSVy = (self.pVy1 + self.pVy2 * self.fnorm(f_z) + ((self.pVy3 + self.pVy4 * self.fnorm(f_z)) * gamma)) * f_z

        # Effective slip angle
        cSAy = alpha + cSHy

        # Shape factors
        cKy = self.pKy1 * self.fnomin * math.sin(2 * math.atan(f_z / (self.pKy2 * self.fnomin))) * (1 - self.pKy3 * math.fabs(gamma))
        cCy = self.pCy1
        cDy = f_z * (self.pDy1 + self.pDy2 * self.fnorm(f_z)) * (1 - self.pDy3 * gamma * gamma)
        cEy = (self.pEy1 + self.pEy2 * self.fnorm(f_z)) * (1 - (self.pEy4 * gamma + self.pEy3) * math.copysign(1, cSAy))
        if cEy > 1:
            cEy = 1

        cBy = cKy/(cCy*cDy)

        # Base lateral force
        fy0 = cDy * math.sin(cCy * math.atan(cBy * cSAy - cEy * (cBy * cSAy - math.atan(cBy * cSAy)))) + cSVy

        # Combined weighing factor
        cCyk = self.rCy1
        cSHyk = self.rHy1 + self.rHy2 * self.fnorm(f_z)
        cByk = self.rBy1 * math.cos(math.atan(self.rBy2 * (alpha - self.rBy3)))
        cDVyk = cDy * (self.rVy1 + self.rVy2 * self.fnorm(f_z) + self.rVy3 * gamma) * math.cos(math.atan(self.rVy4 * alpha))
        cSVyk = cDVyk * math.sin(self.rVy5 * math.atan(self.rVy6 * kappa))
        cEyk = self.rEy1 + self.rEy2 * self.fnorm(f_z)
        if cEyk > 1:
            cEyk = 1

        cGyk = math.cos(cCyk * math.atan(cByk * kappa - cEyk * (cByk * kappa - math.atan(cByk * kappa)))) / math.cos(cCyk * math.atan(cByk * cSHyk - cEyk * (cByk * cSHyk - math.atan(cByk * cSHyk))))

        return cGyk * fy0 + cSVyk

    def calc_mz(self, fz, alpha, gamma, kappa, fy):
        # Pneumatic trail
        cSHt = self.qHz1 + self.qHz2 * self.fnorm(fz) + (self.qHz3 + self.qHz4 * self.fnorm(fz))

        cSAt = alpha + cSHt

        cDt = (fz * self.qDz1 + self.qDz2 * self.fnorm(fz)) * (1 + self.qDz3 * gamma + self.qDz4 * gamma * gamma) * (self.r0 / self.fnomin)
        cCt = self.qCz1
        cBt = (self.qBz1 + self.qBz2 * self.fnorm(fz) + self.qBz3 * self.fnorm(fz) ** 2) * (1 + self.qBz4 + self.qBz5 * math.fabs(gamma))
        cEt = (self.qEz1 + self.qEz2 * self.fnorm(fz) + self.qEz3 * self.fnorm(fz) ** 2) * (1 + (self.qEz5 * gamma) * math.atan(cBt * cCt * cSAt))

        t = cDt * math.cos(cCt * math.atan(cBt * cSAt - cEt * (cBt * cSAt - math.atan(cBt * cSAt)))) * math.cos(alpha)

        # Residual moment
        # From lateral, find a better way of doing this
        cSHy = self.pHy1 + self.pHy2 * self.fnorm(fz) + self.pHy3 * gamma
        cSVy = (self.pVy1 + self.pVy2 * self.fnorm(fz) + ((self.pVy3 + self.pVy4 * self.fnorm(fz)) * gamma)) * fz
        cSAy = alpha + cSHy
        cKy = self.pKy1 * self.fnomin * math.sin(2 * math.atan(fz / (self.pKy2 * self.fnomin))) * (1 - self.pKy3 * math.fabs(gamma))
        cCy = self.pCy1
        cDy = fz * (self.pDy1 + self.pDy2 * self.fnorm(fz)) * (1 - self.pDy3 * gamma * gamma)
        cEy = (self.pEy1 + self.pEy2 * self.fnorm(fz)) * (1 - (self.pEy4 * gamma + self.pEy3) * math.copysign(1, cSAy))
        if cEy > 1:
            cEy = 1
        cBy = cKy/(cCy*cDy)

        cSHf = cSHy + cSVy / cKy

        cSAr = alpha + cSHf

        cBr = self.qBz9 + self.qBz10 * cBy * cCy
        cDr = fz * ((self.qDz6 + self.qDz7 * self.fnorm(fz)) + (self.qDz8 + self.qDz9 * self.fnorm(fz) * gamma)) * self.r0

        cMzr = cDr * math.cos(math.atan(cBr * cSAr)) * math.cos(alpha)

        # Pure side slip for now
        return -t * fy + cMzr
