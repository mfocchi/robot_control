import unittest
from base_controllers.doretta.controllers.lyapunov import LyapunovController, LyapunovParams
from scipy.optimize import fsolve
import logging
log = logging.getLogger(__name__)


class TestStringMethods(unittest.TestCase):

    def test_fsolve(self):
        logging.basicConfig(level=logging.DEBUG)

        params = LyapunovParams(K_P=5., K_THETA=5., DT=0.01)
        controller = LyapunovController(params=params)

        psi =  -2.3562
        eth= -0.6000
        exy= 1.4142
        th= -0.1000
        alpha_d= 0
        beta= 0.4000
        vd= 0
        omegad= 0
        vdot_d= 0.0333
        omegadot_d= 0.0500

        dv0 =   4.4759
        domega0 =   2.8232

        params = (psi, eth, exy, th, alpha_d, beta, vd, omegad, vdot_d, omegadot_d)
        alpha, dv, domega = fsolve(controller.equations, (alpha_d, dv0, domega0), args=params)
        log.debug(f"alpha: {alpha}, dv: {dv},domega: {domega}")

        self.assertAlmostEqual(alpha, 0.0032)
        self.assertAlmostEqual(dv, 4.4932)
        self.assertAlmostEqual(domega, 2.8232)

if __name__ == '__main__':
    unittest.main()
