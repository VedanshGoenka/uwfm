import math


class PacejkaMF52:
    ''' Pacejka MF5.2 tire model.
        http://www.edy.es/dev/docs/pacejka-94-parameters-explained-a-comprehensive-guide/
        http://www.optimumg.com/docs/OptimumTire_Help_File.pdf
    '''

    def __init__(self, coeff):
        # Tire Coefficients
        self.fnomin = coeff['fnomin']
        self.re = coeff['re']
        self.pCy1 = coeff['pcy1']
        self.pDy1 = coeff['pdy1']
        self.pDy2 = coeff['pdy2']
        self.pDy3 = coeff['pdy3']
        self.pDy4 = coeff['pdy4']
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
        self.qBz1 = coeff['qbz1']
        self.qBz2 = coeff['qbz2']
        self.qBz3 = coeff['qbz3']
        self.qBz4 = coeff['qbz4']
        self.qBz5 = coeff['qbz5']
        self.qBz9 = coeff['qbz9']
        self.qBz10 = coeff['qbz10']
        self.qCz1 = coeff['qcz1']
        self.qDz1 = coeff['qdz1']
        self.qDz2 = coeff['qdz2']
        self.qDz3 = coeff['qdz3']
        self.qDz4 = coeff['qdz4']
        self.qDz6 = coeff['qdz6']
        self.qDz7 = coeff['qdz7']
        self.qDz8 = coeff['qdz8']
        self.qDz9 = coeff['qdz9']
        self.qEz1 = coeff['qez1']
        self.qEz2 = coeff['qez2']
        self.qEz3 = coeff['qez3']
        self.qEz4 = coeff['qez4']
        self.qEz5 = coeff['qez5']
        self.qHz1 = coeff['qhz1']
        self.qHz2 = coeff['qhz2']
        self.qHz3 = coeff['qhz3']
        self.qHz4 = coeff['qhz4']
        self.pCx1 = coeff['pcx1']
        self.pDx1 = coeff['pdx1']
        self.pDx2 = coeff['pdx2']
        self.pDx3 = coeff['pdx3']
        self.pDx4 = coeff['pdx4']
        self.pEx1 = coeff['pex1']
        self.pEx2 = coeff['pex2']
        self.pEx3 = coeff['pex3']
        self.pEx4 = coeff['pex4']
        self.pKx1 = coeff['pkx1']
        self.pKx2 = coeff['pkx2']
        self.pKx3 = coeff['pkx3']
        self.pHx1 = coeff['phx1']
        self.pHx2 = coeff['phx2']
        self.pVx1 = coeff['pvx1']
        self.pVx2 = coeff['pvx2']
        self.qSx1 = coeff['qsx1']
        self.qSx2 = coeff['qsx2']
        self.qSx3 = coeff['qsx3']
        self.rCx1 = coeff['rcx1']
        self.rBx1 = coeff['rbx1']
        self.rBx2 = coeff['rbx2']
        self.rHx1 = coeff['rhx1']
        self.rEx1 = coeff['rex1']
        self.rEx2 = coeff['rex2']
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
        self.sSz1 = coeff['ssz1']
        self.sSz2 = coeff['ssz2']
        self.sSz3 = coeff['ssz3']
        self.sSz4 = coeff['ssz4']
        self.r0 = coeff['r0']

    def fnorm(self, f_z):
        '''
        Calculates normalized vertical load, df_z
        where f_z    vertical tire load [N]
        '''
        return (f_z - self.fnomin) / self.fnomin

    def params_fy(self, f_z, alpha, kappa, gamma):
        '''
        Calculates magic formula parameters for lateral force calculations, F_y
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
        cKy = self.pKy1 * self.fnomin * math.sin(2 * math.atan(f_z / (self.pKy2 * self.fnomin))) * (1 - self.pKy3 * gamma ** 2 * math.fabs(gamma))
        cCy = self.pCy1
        cDy = f_z * (self.pDy1 + self.pDy2 * self.fnorm(f_z)) * (1 - self.pDy3 * gamma * gamma)
        cEy = (self.pEy1 + self.pEy2 * self.fnorm(f_z)) * (1 - (self.pEy4 * gamma + self.pEy3) * math.copysign(1, cSAy))
        if cEy > 1:
            cEy = 1

        cBy = cKy/(cCy*cDy)

        # Prepare return structure
        params = {'cSHy': cSHy, 'cSVy': cSVy, 'cSAy': cSAy, 'cKy': cKy,
                  'cCy': cCy, 'cDy': cDy, 'cEy': cEy, 'cBy': cBy}

        return params

    def calc_fy(self, f_z, alpha, kappa, gamma):
        '''
        Calculates tire force, F_y
        where f_z    vertical tire load [N]
              alpha  slip angle [rad]
              gamma  camber angle [rad]
              kappa  slip ratio
        '''
        # Calculate lateral parameters
        params = self.params_fy(f_z, alpha, kappa, gamma)

        # Base lateral force
        fy0 = params['cDy'] \
            * math.sin(params['cCy'] * math.atan(params['cBy'] * params['cSAy'] - params['cEy']
                       * (params['cBy'] * params['cSAy'] - math.atan(params['cBy'] * params['cSAy'])))) \
            + params['cSVy']

        # Combined weighing factor
        cCyk = self.rCy1
        cSHyk = self.rHy1 + self.rHy2 * self.fnorm(f_z)
        cByk = self.rBy1 * math.cos(math.atan(self.rBy2 * (alpha - self.rBy3)))
        cDVyk = params['cDy'] * (self.rVy1 + self.rVy2 * self.fnorm(f_z) + self.rVy3 * gamma) * math.cos(math.atan(self.rVy4 * alpha))
        cSVyk = cDVyk * math.sin(self.rVy5 * math.atan(self.rVy6 * kappa))
        cEyk = self.rEy1 + self.rEy2 * self.fnorm(f_z)
        if cEyk > 1:
            cEyk = 1

        cGyk = math.cos(cCyk * math.atan(cByk * kappa - cEyk * (cByk * kappa - math.atan(cByk * kappa)))) / math.cos(cCyk * math.atan(cByk * cSHyk - cEyk * (cByk * cSHyk - math.atan(cByk * cSHyk))))

        return cGyk * fy0 + cSVyk

    def calc_mz(self, fz, alpha, kappa, gamma):
        # Pneumatic trail
        fy = self.calc_fy(fz, alpha, gamma, kappa)

        cSHt = self.qHz1 + self.qHz2 * self.fnorm(fz) + (self.qHz3 + self.qHz4 * self.fnorm(fz)) * gamma
        cSAt = alpha + cSHt

        cDt = fz * (self.qDz1 + self.qDz2 * self.fnorm(fz)) * (1 + self.qDz3 * gamma + self.qDz4 * gamma * gamma) * (self.r0 / self.fnomin)
        cCt = self.qCz1
        cBt = (self.qBz1 + self.qBz2 * self.fnorm(fz) + self.qBz3 * self.fnorm(fz) ** 2) * (1 + self.qBz4 * gamma + self.qBz5 * math.fabs(gamma))
        if cBt < 0:
            cBt = 0

        cEt = (self.qEz1 + self.qEz2 * self.fnorm(fz) + self.qEz3 * self.fnorm(fz) ** 2) * (1 + (self.qEz4 + self.qEz5 * gamma) * math.atan(cBt * cCt * cSAt))
        if cEt > 1:
            cEt = 1

        t = cDt * math.cos(cCt * math.atan(cBt * cSAt - cEt * (cBt * cSAt - math.atan(cBt * cSAt)))) * math.cos(alpha)

        # Residual moment
        # This is highly inefficient since we are going through the same calculation multiple times
        param_y = self.params_fy(fz, alpha, gamma, kappa)

        cSHf = param_y['cSHy'] + param_y['cSVy'] / param_y['cKy']
        cSAr = alpha + cSHf

        cBr = self.qBz9 + self.qBz10 * param_y['cBy'] * param_y['cCy']
        cDr = fz * ((self.qDz6 + self.qDz7 * self.fnorm(fz)) + (self.qDz8 + self.qDz9 * self.fnorm(fz)) * gamma) * self.r0 * math.cos(alpha)

        cMzr = cDr * math.cos(math.atan(cBr * cSAr))

        # Pure side slip for now
        return -t * fy + cMzr
