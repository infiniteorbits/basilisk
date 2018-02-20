''' '''
'''
 ISC License

 Copyright (c) 2016-2018, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

'''
import sys, os, inspect
import pytest
import matplotlib.pyplot as plt
import numpy as np

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
from Basilisk.simulation import spacecraftPlus
from Basilisk.simulation import fuelSloshParticle
from Basilisk.simulation import fuelTank
from Basilisk.simulation import gravityEffector
from Basilisk.utilities import macros

@pytest.mark.parametrize("useFlag, testCase", [
    (False,'NoGravity'),
    (False,'Gravity'),
    (False,'Damping')
])

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_
def test_fuelSloshAllTest(show_plots,useFlag,testCase):
    [testResults, testMessage] = fuelSloshTest(show_plots,useFlag,testCase)
    assert testResults < 1, testMessage

def fuelSloshTest(show_plots,useFlag,testCase):
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages
    
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"
    
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)
    
    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()
    unitTestSim.TotalSim.terminateSimulation()
    
    # Create test thread
    testProcessRate = macros.sec2nano(0.001)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    unitTestSim.particle1 = fuelSloshParticle.FuelSloshParticle()
    unitTestSim.particle2 = fuelSloshParticle.FuelSloshParticle()
    unitTestSim.particle3 = fuelSloshParticle.FuelSloshParticle()

    # Define Variables for particle 1
    unitTestSim.particle1.k = 100.0
    unitTestSim.particle1.c = 0.0
    unitTestSim.particle1.r_PB_B = [[0.1], [0], [-0.1]]
    unitTestSim.particle1.pHat_B = [[np.sqrt(3)/3], [np.sqrt(3)/3], [np.sqrt(3)/3]]
    unitTestSim.particle1.nameOfRhoState = "fuelSloshParticleRho1"
    unitTestSim.particle1.nameOfRhoDotState = "fuelSloshParticleRhoDot1"
    unitTestSim.particle1.nameOfMassState = "fuelSloshParticleMass1"
    unitTestSim.particle1.rhoInit = 0.05
    unitTestSim.particle1.rhoDotInit = 0.0
    unitTestSim.particle1.massInit = 10.0

    # Define Variables for particle 2
    unitTestSim.particle2.k = 100.0
    unitTestSim.particle2.c = 0.0
    unitTestSim.particle2.r_PB_B = [[0], [0], [0.1]]
    unitTestSim.particle2.pHat_B = [[np.sqrt(3)/3], [-np.sqrt(3)/3], [-np.sqrt(3)/3]]
    unitTestSim.particle2.nameOfRhoState = "fuelSloshParticleRho2"
    unitTestSim.particle2.nameOfRhoDotState = "fuelSloshParticleRhoDot2"
    unitTestSim.particle2.nameOfMassState = "fuelSloshParticleMass2"
    unitTestSim.particle2.rhoInit = -0.025
    unitTestSim.particle2.rhoDotInit = 0.0
    unitTestSim.particle2.massInit = 20.0

    # Define Variables for particle 3
    unitTestSim.particle3.k = 100.0
    unitTestSim.particle3.c = 0.0
    unitTestSim.particle3.r_PB_B = [[-0.1], [0], [0.1]]
    unitTestSim.particle3.pHat_B = [[-np.sqrt(3)/3], [-np.sqrt(3)/3], [np.sqrt(3)/3]]
    unitTestSim.particle3.nameOfRhoState = "fuelSloshParticleRho3"
    unitTestSim.particle3.nameOfRhoDotState = "fuelSloshParticleRhoDot3"
    unitTestSim.particle3.nameOfMassState = "fuelSloshParticleMass3"
    unitTestSim.particle3.rhoInit = -0.015
    unitTestSim.particle3.rhoDotInit = 0.0
    unitTestSim.particle3.massInit = 15.0

    if testCase == 'Damping':
        unitTestSim.particle1.c = 15.0
        unitTestSim.particle2.c = 17.0
        unitTestSim.particle3.c = 11.0

    #define the fuel tank
    unitTestSim.tank1 = fuelTank.FuelTank()
    unitTestSim.tank1.setTankModel(fuelTank.TANK_MODEL_CONSTANT_VOLUME)
    tankModel = fuelTank.cvar.FuelTankModelConstantVolume
    tankModel.propMassInit = 30.0
    tankModel.r_TcT_TInit = [[0.0],[0.0],[0.0]]
    tankModel.radiusTankInit = 0.5
    unitTestSim.tank1.r_TB_B = [[0],[0],[0.1]]
    unitTestSim.tank1.nameOfMassState = "fuelTankMass1"
    unitTestSim.tank1.pushFuelSloshParticle(unitTestSim.particle1)
    unitTestSim.tank1.pushFuelSloshParticle(unitTestSim.particle2)
    unitTestSim.tank1.pushFuelSloshParticle(unitTestSim.particle3)
    unitTestSim.tank1.updateOnly = True

    # Add panels to spaceCraft
    # this next line is not working
    scObject.addStateEffector(unitTestSim.tank1)
    
    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    scObject.hub.mHub = 750
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]
    scObject.hub.r_CN_NInit = [[0.5], [0.4], [-0.7]]
    scObject.hub.v_CN_NInit = [[0.1], [0.-5], [0.3]]
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[0.1], [-0.1], [0.1]]

    if testCase == 'Gravity':
        unitTestSim.earthGravBody = gravityEffector.GravBodyData()
        unitTestSim.earthGravBody.bodyInMsgName = "earth_planet_data"
        unitTestSim.earthGravBody.outputMsgName = "earth_display_frame_data"
        unitTestSim.earthGravBody.mu = 0.3986004415E+15 # meters!
        unitTestSim.earthGravBody.isCentralBody = True
        unitTestSim.earthGravBody.useSphericalHarmParams = False
        scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector([unitTestSim.earthGravBody])
        scObject.hub.r_CN_NInit = [[-4020338.690396649],	[7490566.741852513],	[5248299.211589362]]
        scObject.hub.v_CN_NInit = [[-5199.77710904224],	[-3436.681645356935],	[1041.576797498721]]

    unitTestSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, testProcessRate)

    unitTestSim.InitializeSimulation()

    unitTestSim.AddVariableForLogging(scObject.ModelTag + ".totOrbEnergy", testProcessRate, 0, 0, 'double')
    unitTestSim.AddVariableForLogging(scObject.ModelTag + ".totOrbAngMomPntN_N", testProcessRate, 0, 2, 'double')
    unitTestSim.AddVariableForLogging(scObject.ModelTag + ".totRotAngMomPntC_N", testProcessRate, 0, 2, 'double')
    unitTestSim.AddVariableForLogging(scObject.ModelTag + ".totRotEnergy", testProcessRate, 0, 0, 'double')

    posRef = scObject.dynManager.getStateObject("hubPosition")
    sigmaRef = scObject.dynManager.getStateObject("hubSigma")

    stopTime = 2.5
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    orbEnergy = unitTestSim.GetLogVariableData(scObject.ModelTag + ".totOrbEnergy")
    orbAngMom_N = unitTestSim.GetLogVariableData(scObject.ModelTag + ".totOrbAngMomPntN_N")
    rotAngMom_N = unitTestSim.GetLogVariableData(scObject.ModelTag + ".totRotAngMomPntC_N")
    rotEnergy = unitTestSim.GetLogVariableData(scObject.ModelTag + ".totRotEnergy")

    initialOrbAngMom_N = [
                [orbAngMom_N[0,1], orbAngMom_N[0,2], orbAngMom_N[0,3]]
                ]

    finalOrbAngMom = [
                [orbAngMom_N[-1,0], orbAngMom_N[-1,1], orbAngMom_N[-1,2], orbAngMom_N[-1,3]]
                 ]

    initialRotAngMom_N = [
                [rotAngMom_N[0,1], rotAngMom_N[0,2], rotAngMom_N[0,3]]
                ]

    finalRotAngMom = [
                [rotAngMom_N[-1,0], rotAngMom_N[-1,1], rotAngMom_N[-1,2], rotAngMom_N[-1,3]]
                 ]

    initialOrbEnergy = [
                [orbEnergy[0,1]]
                ]

    finalOrbEnergy = [
                [orbEnergy[-1,0], orbEnergy[-1,1]]
                 ]

    initialRotEnergy = [
                [rotEnergy[0,1]]
                ]

    finalRotEnergy = [
                [rotEnergy[-1,0], rotEnergy[-1,1]]
                 ]

    plt.figure()
    plt.clf()
    plt.plot(orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,1] - orbAngMom_N[0,1])/orbAngMom_N[0,1], orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,2] - orbAngMom_N[0,2])/orbAngMom_N[0,2], orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,3] - orbAngMom_N[0,3])/orbAngMom_N[0,3])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    unitTestSupport.writeFigureLaTeX("ChangeInOrbitalAngularMomentum" + testCase, "Change in Orbital Angular Momentum " + testCase, plt, "width=0.8\\textwidth", path)
    plt.figure()
    plt.clf()
    plt.plot(orbEnergy[:,0]*1e-9, (orbEnergy[:,1] - orbEnergy[0,1])/orbEnergy[0,1])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    unitTestSupport.writeFigureLaTeX("ChangeInOrbitalEnergy" + testCase, "Change in Orbital Energy " + testCase, plt, "width=0.8\\textwidth", path)
    plt.figure()
    plt.clf()
    plt.plot(rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,1] - rotAngMom_N[0,1])/rotAngMom_N[0,1], rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,2] - rotAngMom_N[0,2])/rotAngMom_N[0,2], rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,3] - rotAngMom_N[0,3])/rotAngMom_N[0,3])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    unitTestSupport.writeFigureLaTeX("ChangeInRotationalAngularMomentum" + testCase, "Change in Rotational Angular Momentum " + testCase, plt, "width=0.8\\textwidth", path)
    if testCase == 'Gravity' or testCase == 'NoGravity':
        plt.figure()
        plt.clf()
        plt.plot(rotEnergy[:,0]*1e-9, (rotEnergy[:,1] - rotEnergy[0,1])/rotEnergy[0,1])
        plt.xlabel("Time (s)")
        plt.ylabel("Relative Difference")
        unitTestSupport.writeFigureLaTeX("ChangeInRotationalEnergy" + testCase, "Change in Rotational Energy " + testCase, plt, "width=0.8\\textwidth", path)

    plt.show(show_plots)

    accuracy = 1e-10
    for i in range(0,len(initialOrbAngMom_N)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalOrbAngMom[i],initialOrbAngMom_N[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Slosh unit test failed orbital angular momentum unit test")

    for i in range(0,len(initialRotAngMom_N)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalRotAngMom[i],initialRotAngMom_N[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Slosh unit test failed rotational angular momentum unit test")

    if testCase == 'Gravity' or testCase == 'NoGravity':
        for i in range(0,len(initialRotEnergy)):
            # check a vector values
            if not unitTestSupport.isArrayEqualRelative(finalRotEnergy[i],initialRotEnergy[i],1,accuracy):
                testFailCount += 1
                testMessages.append("FAILED: Fuel Slosh unit test failed rotational energy unit test")

    for i in range(0,len(initialOrbEnergy)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalOrbEnergy[i],initialOrbEnergy[i],1,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Slosh unit test failed orbital energy unit test")

    if testFailCount == 0:
        print "PASSED: " + " Fuel Slosh Test"

    assert testFailCount < 1, testMessages
    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

if __name__ == "__main__":
    fuelSloshTest(True,False,'Damping')