#
#  ISC License
#
#  Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#


#
#   Unit Test Script
#   Module Name:        relativeNav
#   Author:             Enrique Carlos Toomey
#   Creation Date:      Sep 02 2024
#

import pytest
import numpy as np
from Basilisk.architecture import messaging
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import macros
from Basilisk.simulation import relativeNav


def test_without_noise():
    r_BsN_N = [6800000.0, 0.0, 0.0]
    v_BsN_N = [0.0, 7800.0, 0.0]
    r_BcN_N = [6810000.0, 0.0, 0.0]
    v_BcN_N = [0.0, 7750.0, 0.0]
    sigma_BsN = [0, 0, 0]
    omega_BsN_Bs = [0, 0, 0]
    sigma_BcN = [0, 0, 0]
    omega_BcN_Bc = [0, 0, 0]
    posSigma = velSigma = attSigma = rateSigma = 0
    r_BcBs_Bs_expected = [10000.0, 0.0, 0.0]
    v_BcBs_Bs_expected = [0.0, -50.0, 0.0]
    sigma_BcBs_expected = [0, 0, 0]
    omega_BcBs_Bs_expected = np.cross(r_BcBs_Bs_expected, omega_BsN_Bs)

    relativeNav_test_function(r_BsN_N, v_BsN_N, sigma_BsN, omega_BsN_Bs,
                              r_BcN_N, v_BcN_N, sigma_BcN, omega_BcN_Bc,
                              posSigma, velSigma, attSigma, rateSigma,
                              r_BcBs_Bs_expected, v_BcBs_Bs_expected,
                              sigma_BcBs_expected, omega_BcBs_Bs_expected)
#posBound = numpy.array([1000.0] * 3)
#velBound = numpy.array([1.0] * 3)
#attBound = numpy.array([5E-3] * 3)
#rateBound = numpy.array([0.02] * 3)

#posSigma = 5.0
#velSigma = 0.035
#attSigma = 1.0 / 360.0 * np.pi / 180.0
#rateSigma = 0.05 * np.pi / 180.0
#errorBounds = [[1000.], [1000.], [1000.], [1.], [1.], [1.], [0.005], [0.005], [0.005], [0.02], [0.02], [0.02]]




def relativeNav_test_function(r_BsN_N, v_BsN_N, sigma_BsN, omega_BsN_Bs,
                              r_BcN_N, v_BcN_N, sigma_BcN, omega_BcN_Bc,
                              posSigma, velSigma, attSigma, rateSigma,
                              r_BcBs_Bs_expected, v_BcBs_Bs_expected,
                              sigma_BcBs_expected, omega_BcBs_Bs_expected):
    # Create a sim module as an empty container
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(0.5)
    unitTestProc = unitTestSim.CreateNewProcess(unitProcessName)
    # create the task and specify the integration update time
    unitTestProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    #Now initialize the modules that we are using.  I got a little better as I went along
    rNavObject = relativeNav.RelativeNav()
    unitTestSim.AddModelToTask(unitTaskName, rNavObject)

    servicerStateMessage = messaging.SCStatesMsgPayload()
    servicerStateMessage.r_BN_N = r_BsN_N
    servicerStateMessage.v_BN_N = v_BsN_N
    servicerStateMessage.sigma_BN = sigma_BsN
    servicerStateMessage.omega_BN_B = omega_BsN_Bs
    
    clientStateMessage = messaging.SCStatesMsgPayload()
    clientStateMessage.r_BN_N = r_BcN_N
    clientStateMessage.v_BN_N = v_BcN_N
    clientStateMessage.sigma_BN = sigma_BcN
    clientStateMessage.omega_BN_B = omega_BcN_Bc


    # Inertial State output Message
    servicerStateMsg = messaging.SCStatesMsg().write(servicerStateMessage)
    rNavObject.servicerStateInMsg.subscribeTo(servicerStateMsg)
    clientStateMsg = messaging.SCStatesMsg().write(clientStateMessage)
    rNavObject.clientStateInMsg.subscribeTo(clientStateMsg)

    rNavObject.ModelTag = "RelativeNavigation"
    pMatrix = [[posSigma, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
               [0., posSigma, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
               [0., 0., posSigma, 0., 0., 0., 0., 0., 0., 0., 0., 0.],
               [0., 0., 0., velSigma, 0., 0., 0., 0., 0., 0., 0., 0.],
               [0., 0., 0., 0., velSigma, 0., 0., 0., 0., 0., 0., 0.],
               [0., 0., 0., 0., 0., velSigma, 0., 0., 0., 0., 0., 0.],
               [0., 0., 0., 0., 0., 0., attSigma, 0., 0., 0., 0., 0.],
               [0., 0., 0., 0., 0., 0., 0., attSigma, 0., 0., 0., 0.],
               [0., 0., 0., 0., 0., 0., 0., 0., attSigma, 0., 0., 0.],
               [0., 0., 0., 0., 0., 0., 0., 0., 0., rateSigma, 0., 0.],
               [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., rateSigma, 0.],
               [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., rateSigma],
               ]
    #rNavObject.walkBounds = errorBounds
    rNavObject.PMatrix = pMatrix
    rNavObject.crossTrans = True
    rNavObject.crossAtt = False

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(testProcessRate)
    unitTestSim.ExecuteSimulation()

    r_BcBs_Bs = rNavObject.relTransOutMsg.read().r_BcBc_Bs
    v_BcBc_Bs = rNavObject.relTransOutMsg.read().v_BcBc_Bs
    sigma_BcBs = rNavObject.relAttOutMsg.read().sigma_BcBs
    omega_BcBs_Bs = rNavObject.relAttOutMsg.read().omega_BcBs_Bs
    print(r_BcBs_Bs, v_BcBc_Bs, sigma_BcBs, omega_BcBs_Bs)
    rta = unitTestSupport.isVectorEqual(r_BcBs_Bs, r_BcBs_Bs_expected, 1e-3)
    rta = unitTestSupport.isVectorEqual(v_BcBc_Bs, v_BcBs_Bs_expected, 1e-3)
    rta = unitTestSupport.isVectorEqual(sigma_BcBs, sigma_BcBs_expected, 1e-3)
    rta = unitTestSupport.isVectorEqual(omega_BcBs_Bs, omega_BcBs_Bs_expected, 1e-3)
    assert rta > 0, f"""Test failed: 
        expected={r_BcBs_Bs_expected}, result={r_BcBs_Bs},
        expected={v_BcBs_Bs_expected}, result={r_BcBs_Bs},
        expected={sigma_BcBs_expected}, result={sigma_BcBs},
        expected={omega_BcBs_Bs_expected}, result={omega_BcBs_Bs},
        """

if __name__ == "__main__":
    test_without_noise()

