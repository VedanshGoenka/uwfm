import math


class PacejkaMF52:
    '''
    A class to represent the Magic Formula 5.2 tire model. Equations
    are based on Pacejka 2002 with an extra term added into the
    longitudinal calculations.

    References
    H. B. Pacejka, 'Tire and Vehicle Dynamics', 2006
    http://www.racer.nl/reference/pacejka.htm
    http://www.optimumg.com/docs/OptimumTire_Help_File.pdf
    '''

    def __init__(self, general, u, py, qz, px, qx, rx, ry, sz, qy):
        '''
        Initialize tire coefficients. Coefficients are categorized by
        type as per the OptimumTire documentation. The categeories are
        shortened to the first and third letter of the coefficient
        names. For example, coefficients in the pure lateral group are
        named p.y##, therefore the group is shortened to simply 'py'.

        Inputs
        general: general data
              u: scaling factors
             py: pure lateral
             qz: aligning torque
             px: pure lognitudinal
             qx: overturning moment
             rx: combined longitudinal
             ry: combined lateral
             sz: combined aligning torque
             qy: roll moment
        '''
        self.ux = u['ux']
        self.uy = u['uy']

        self.fnomin = general['fnomin']
        self.re = general['re']
        self.r0 = general['r0']
        self.pCy1 = py['pcy1']
        self.pDy1 = py['pdy1']
        self.pDy2 = py['pdy2']
        self.pDy3 = py['pdy3']
        self.pDy4 = py['pdy4']
        self.pEy1 = py['pey1']
        self.pEy2 = py['pey2']
        self.pEy3 = py['pey3']
        self.pEy4 = py['pey4']
        self.pKy1 = py['pky1']
        self.pKy2 = py['pky2']
        self.pKy3 = py['pky3']
        self.pHy1 = py['phy1']
        self.pHy2 = py['phy2']
        self.pHy3 = py['phy3']
        self.pVy1 = py['pvy1']
        self.pVy2 = py['pvy2']
        self.pVy3 = py['pvy3']
        self.pVy4 = py['pvy4']
        self.qBz1 = qz['qbz1']
        self.qBz2 = qz['qbz2']
        self.qBz3 = qz['qbz3']
        self.qBz4 = qz['qbz4']
        self.qBz5 = qz['qbz5']
        self.qBz9 = qz['qbz9']
        self.qBz10 = qz['qbz10']
        self.qCz1 = qz['qcz1']
        self.qDz1 = qz['qdz1']
        self.qDz2 = qz['qdz2']
        self.qDz3 = qz['qdz3']
        self.qDz4 = qz['qdz4']
        self.qDz6 = qz['qdz6']
        self.qDz7 = qz['qdz7']
        self.qDz8 = qz['qdz8']
        self.qDz9 = qz['qdz9']
        self.qEz1 = qz['qez1']
        self.qEz2 = qz['qez2']
        self.qEz3 = qz['qez3']
        self.qEz4 = qz['qez4']
        self.qEz5 = qz['qez5']
        self.qHz1 = qz['qhz1']
        self.qHz2 = qz['qhz2']
        self.qHz3 = qz['qhz3']
        self.qHz4 = qz['qhz4']
        self.pCx1 = px['pcx1']
        self.pDx1 = px['pdx1']
        self.pDx2 = px['pdx2']
        self.pDx3 = px['pdx3']
        self.pDx4 = px['pdx4']
        self.pEx1 = px['pex1']
        self.pEx2 = px['pex2']
        self.pEx3 = px['pex3']
        self.pEx4 = px['pex4']
        self.pKx1 = px['pkx1']
        self.pKx2 = px['pkx2']
        self.pKx3 = px['pkx3']
        self.pHx1 = px['phx1']
        self.pHx2 = px['phx2']
        self.pVx1 = px['pvx1']
        self.pVx2 = px['pvx2']
        self.qSx1 = qx['qsx1']
        self.qSx2 = qx['qsx2']
        self.qSx3 = qx['qsx3']
        self.rCx1 = rx['rcx1']
        self.rBx1 = rx['rbx1']
        self.rBx2 = rx['rbx2']
        self.rHx1 = rx['rhx1']
        self.rEx1 = rx['rex1']
        self.rEx2 = rx['rex2']
        self.rCy1 = ry['rcy1']
        self.rBy1 = ry['rby1']
        self.rBy2 = ry['rby2']
        self.rBy3 = ry['rby3']
        self.rHy1 = ry['rhy1']
        self.rHy2 = ry['rhy2']
        self.rVy1 = ry['rvy1']
        self.rVy2 = ry['rvy2']
        self.rVy3 = ry['rvy3']
        self.rVy4 = ry['rvy4']
        self.rVy5 = ry['rvy5']
        self.rVy6 = ry['rvy6']
        self.rEy1 = ry['rey1']
        self.rEy2 = ry['rey2']
        self.sSz1 = sz['ssz1']
        self.sSz2 = sz['ssz2']
        self.sSz3 = sz['ssz3']
        self.sSz4 = sz['ssz4']

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
        cSVy = (self.pVy1 + self.pVy2 * self.fnorm(f_z) + ((self.pVy3 + self.pVy4 * self.fnorm(f_z)) * gamma)) * f_z * self.uy

        # Effective slip angle
        cSAy = alpha + cSHy

        # Shape factors
        cKy = self.pKy1 * self.fnomin * math.sin(2 * math.atan(f_z / (self.pKy2 * self.fnomin))) * (1 - self.pKy3 * gamma ** 2 * math.fabs(gamma))
        cCy = self.pCy1
        cDy = f_z * self.uy * (self.pDy1 + self.pDy2 * self.fnorm(f_z)) * (1 - self.pDy2 * gamma * gamma)
        cEy = (self.pEy1 + self.pEy2 * self.fnorm(f_z)) * (1 - (self.pEy4 * gamma + self.pEy3) * math.copysign(1, cSAy))
        if cEy > 1:
            cEy = 1

        try:
            cBy = cKy/(cCy*cDy)
        except ZeroDivisionError:
            cBy = 0

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
        if f_z < 0:
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

            cGyk = math.cos(cCyk * math.atan(cByk * kappa - cEyk * (cByk * kappa - math.atan(cByk * kappa)))) \
                / math.cos(cCyk * math.atan(cByk * cSHyk - cEyk * (cByk * cSHyk - math.atan(cByk * cSHyk))))

            fy = cGyk * fy0 + cSVyk

        else:
            fy = 0

        return fy

    def calc_mz(self, fz, alpha, kappa, gamma):
        if fz < 0:
            # Pneumatic trail
            fy = self.calc_fy(fz, alpha, gamma, kappa)

            cSHt = self.qHz1 + self.qHz2 * self.fnorm(fz) + (self.qHz3 + self.qHz4 * self.fnorm(fz)) * gamma
            cSAt = alpha + cSHt

            cDt = fz * (self.qDz1 + self.qDz2 * self.fnorm(fz)) * (1 + self.qDz3 * gamma + self.qDz4 * gamma * gamma) * (self.r0 / self.fnomin)
            cCt = self.qCz1
            cBt = (self.qBz1 + self.qBz2 * self.fnorm(fz) + self.qBz3 * self.fnorm(fz) ** 2) * (1 + self.qBz4 * gamma + self.qBz5 * math.fabs(gamma)) / self.uy
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

            cBr = self.qBz9/self.uy + self.qBz10 * param_y['cBy'] * param_y['cCy']
            cDr = fz * ((self.qDz6 + self.qDz7 * self.fnorm(fz)) + (self.qDz8 + self.qDz9 * self.fnorm(fz)) * gamma) * self.r0 * math.cos(alpha) * self.uy

            cMzr = cDr * math.cos(math.atan(cBr * cSAr))

            # Pure side slip for now
            mz = -t * fy + cMzr

        else:
            mz = 0

        return mz
