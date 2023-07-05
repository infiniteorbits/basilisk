#
#  ISC License
#
#  Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

r"""
Overview
--------

This scenario is an exact replica of :ref:`scenarioAttitudeFeedback2T_TH`. The only difference lies in the fact that
this scenario uses the :ref:`thrusterStateEffector` module instead of :ref:`thrusterDynamicEffector`. The performance
and results should be nearly identical to the original scenario, with the small difference that the thrusters do not
have an on-off behavior, but instead behave like a first-order filter. For more information on the scenario setup, see
:ref:`scenarioAttitudeFeedback2T_TH`.

To show that the :ref:`thrusterStateEffector` thruster module works with variable time step integrators, this scenario
uses an RKF78 integrator instead of the usual RK4.

Illustration of Simulation Results
----------------------------------

::

    show_plots = True, useDVThrusters = False

.. image:: /_images/Scenarios/scenarioAttitudeFeedback2T_stateEffTH10.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeFeedback2T_stateEffTH20.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeFeedback2T_stateEffTH30.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeFeedback2T_stateEffTH40.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeFeedback2T_stateEffTH50.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeFeedback2T_stateEffTH60.svg
   :align: center


::

    show_plots = True, useDVThrusters = True

.. image:: /_images/Scenarios/scenarioAttitudeFeedback2T_stateEffTH11.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeFeedback2T_stateEffTH21.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeFeedback2T_stateEffTH31.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeFeedback2T_stateEffTH41.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeFeedback2T_stateEffTH51.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeFeedback2T_stateEffTH61.svg
   :align: center

"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test of the spacecraft(), extForceTorque, simpleNav(), thrusterDynamicEffector() and
#           mrpFeedback() modules.  Illustrates a 6-DOV spacecraft detumbling in orbit, while using thrusters
#           to do the attitude control actuation.
# Author: João Vaz Carneiro
# Creation Date:  July 27, 2022
#

import os

import matplotlib.pyplot as plt
import numpy as np
# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__
# import message declarations
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import attTrackingError
from Basilisk.fswAlgorithms import inertial3D
# import FSW Algorithm related support
from Basilisk.fswAlgorithms import mrpFeedback
from Basilisk.fswAlgorithms import thrFiringSchmitt
from Basilisk.fswAlgorithms import thrForceMapping
from Basilisk.simulation import extForceTorque
from Basilisk.simulation import simpleNav
# import simulation related support
from Basilisk.simulation import spacecraft
from Basilisk.simulation import svIntegrators
from Basilisk.simulation import thrusterStateEffector
from Basilisk.simulation import spinningBodyOneDOFStateEffector
# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import fswSetupThrusters
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import simIncludeThruster
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
from Basilisk.utilities import RigidBodyKinematics
# attempt to import vizard
from Basilisk.utilities import vizSupport

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


# Plotting functions
def plot_thrPosDiff(timedata, thrPosDiff):
    """Plot the difference between thruster position and arm position"""
    plt.figure(3)
    for idx in range(3):
        plt.plot(timedata, thrPosDiff[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'thrPosDiff ' + str(idx))
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Thruster Position Diff')

def plot_(timeDataFSW, dataSigmaBR):
    """Plot the attitude errors."""
    plt.figure(1)
    for idx in range(3):
        plt.plot(timeDataFSW, dataSigmaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_' + str(idx) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Attitude Error $\sigma_{B/R}$')


def plot_rate_error(timeDataFSW, dataOmegaBR):
    """Plot the body angular velocity tracking errors."""
    plt.figure(2)
    for idx in range(3):
        plt.plot(timeDataFSW, dataOmegaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_{BR,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Rate Tracking Error [rad/s] ')


def plot_requested_torque(timeDataFSW, dataLr):
    """Plot the commanded attitude control torque."""
    plt.figure(3)
    for idx in range(3):
        plt.plot(timeDataFSW, dataLr[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$L_{r,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Control Torque $L_r$ [Nm]')


def plot_armTheta(timeData, thetaData, numTh):
    """Plot the spinningOneDOF theta values."""
    plt.figure(1)
    plt.plot(timeData, thetaData)
    plt.xlabel('Time [min]')
    plt.ylabel('Theta [rad]')

def plot_armThetaDot(timeData, thetaDotData, numTh):
    """Plot the spinningOneDOF theta values."""
    plt.figure(2)
    plt.plot(timeData, thetaDotData)
    plt.xlabel('Time [min]')
    plt.ylabel('ThetaDot [rad/s]')


def plot_OnTimeRequest(timeDataFSW, dataSchm, numTh):
    """Plot the thruster on time requests."""
    plt.figure(5)
    for idx in range(numTh):
        plt.plot(timeDataFSW, dataSchm[:, idx],
                 color=unitTestSupport.getLineColor(idx, numTh),
                 label='$OnTimeRequest,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('OnTimeRequest [sec]')


def plot_trueThrForce(timeDataFSW, dataMap, numTh):
    """Plot the Thruster force values."""
    plt.figure(6)
    for idx in range(numTh):
        plt.plot(timeDataFSW, dataMap[:, idx],
                 color=unitTestSupport.getLineColor(idx, numTh),
                 label='$thrForce,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Force implemented[N]')


def run(show_plots, useDVThrusters):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots
        useDVThrusters (bool): Use 6 DV thrusters instead of the default 8 ACS thrusters.

    """

    # Create simulation variable names
    dynTaskName = "dynTask"
    dynProcessName = "dynProcess"

    fswTaskName = "fswTask"
    fswProcessName = "fswProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    # set the simulation time variable used later on
    simulationTime = macros.min2nano(10.)

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(dynProcessName)
    fswProcess = scSim.CreateNewProcess(fswProcessName)

    # create the dynamics task and specify the integration update time
    simTimeStep = macros.sec2nano(0.1)
    dynProcess.addTask(scSim.CreateNewTask(dynTaskName, simTimeStep))
    
    #
    #   setup the simulation tasks/objects
    #

    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bskSat"
    # define the simulation inertia
    I = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]
    scObject.hub.mHub = 750.0  # kg - spacecraft mass
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    # add spacecraft object to the simulation process
    scSim.AddModelToTask(dynTaskName, scObject, ModelPriority=20)

    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()

    # setup Earth Gravity Body
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    mu = earth.mu

    # attach gravity model to spacecraft
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))


    # create the set of thruster in the dynamics task
    thrusterSet = thrusterStateEffector.ThrusterStateEffector()
    scSim.AddModelToTask(dynTaskName, thrusterSet)

    # set the integrator to a variable time step of 7th-8th order
    integratorObject = svIntegrators.svIntegratorRK4(scObject)
    scObject.setIntegrator(integratorObject)

    # Make a fresh thruster factory instance, this is critical to run multiple times
    thFactory = simIncludeThruster.thrusterFactory()

    # setup 1DOF arm to attach thruster onto
    spinningBody1 = spinningBodyOneDOFStateEffector.SpinningBodyOneDOFStateEffector()
    spinningBody1.mass = 100.0
    spinningBody1.IPntSc_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    spinningBody1.dcm_S0B = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    spinningBody1.r_ScS_S = [[0.0], [0.0], [0.0]]
    spinningBody1.r_SB_B = [[0], [2], [0]]
    spinningBody1.sHat_S = [[0], [1], [0]]

    spinningBody1.thetaInit = 3 * macros.D2R
    spinningBody1.thetaDotInit = 2 * macros.D2R

    spinningBody1.k = 1.0
    spinningBody1.c = 0.5

    scObject.addStateEffector(spinningBody1)
    scSim.AddModelToTask(dynTaskName, spinningBody1, ModelPriority=10)

    # create a thruster attached to a spinningBodyOneDOFStateEffector
    long_angle_rad = 30.0 * np.pi / 180.0
    lat_angle_rad = 15.0 * np.pi / 180.0
    thruster1 = thrusterStateEffector.THRSimConfig()
    thruster1.thrLoc_B = np.array([[1.], [0.0], [0.0]]).reshape([3, 1])
    thruster1.thrDir_B = np.array(
        [[np.cos(long_angle_rad + np.pi / 4.) * np.cos(lat_angle_rad - np.pi / 4.)],
            [np.sin(long_angle_rad + np.pi / 4.) * np.cos(lat_angle_rad - np.pi / 4.)],
            [np.sin(lat_angle_rad - np.pi / 4.)]]).reshape([3, 1])
    thruster1.MaxThrust = 20.0
    thruster1.steadyIsp = 226.7
    thruster1.MinOnTime = 0.006
    thruster1.cutoffFrequency = 2
    thruster1.attachedBodyAxis = np.array([[1.],[0.0],[0.0]])
    thrusterSet.addThruster(thruster1, spinningBody1.spinningBodyConfigLogOutMsg)

    # get number of thruster devices
    numTh = 1

    # create thruster object container and tie to spacecraft object
    thrModelTag = "ACSThrusterDynamics"
    thFactory.addToSpacecraft(thrModelTag, thrusterSet, scObject)

    #
    #   Setup data logging before the simulation is initialized
    #
    scStateLog = scObject.scStateOutMsg.recorder()
    scSim.AddModelToTask(dynTaskName,scStateLog)
    armStateLog = spinningBody1.spinningBodyOutMsg.recorder()
    scSim.AddModelToTask(dynTaskName, armStateLog)
    armInertialStateLog = spinningBody1.spinningBodyConfigLogOutMsg.recorder()
    scSim.AddModelToTask(dynTaskName, armInertialStateLog)
    thrStateLog = []
    for i in range(numTh):
        thrStateLog.append(thrusterSet.thrusterOutMsgs[i].recorder())
        scSim.AddModelToTask(dynTaskName, thrStateLog[i])

    #   set initial Spacecraft States
    #
    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    oe.a = 10000000.0  # meters
    oe.e = 0.01
    oe.i = 0 * macros.D2R
    oe.Omega = 90 * macros.D2R
    oe.omega = 90 * macros.D2R
    oe.f = 0 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    scObject.hub.r_CN_NInit = rN  # m   - r_CN_N
    scObject.hub.v_CN_NInit = vN  # m/s - v_CN_N
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
    scObject.hub.omega_BN_BInit = [[0], [0], [0]]  # rad/s - omega_BN_B

    # if this scenario is to interface with the BSK Viz, uncomment the following lines
    viz = vizSupport.enableUnityVisualization(scSim, dynTaskName,  scObject
                                              # , saveFile=fileName
                                              , thrEffectorList=thrusterSet
                                              , thrColors=vizSupport.toRGBA255("red")
                                              )
    vizSupport.setActuatorGuiSetting(viz, showThrusterLabels=True)

    #
    #   initialize Simulation
    #
    scSim.InitializeSimulation()

    # Add energy and momentum variables to log
    scSim.AddVariableForLogging(scObject.ModelTag + ".totRotEnergy", simTimeStep, 0, 0, 'double')
    scSim.AddVariableForLogging(scObject.ModelTag + ".totOrbEnergy", simTimeStep, 0, 0, 'double')
    scSim.AddVariableForLogging(scObject.ModelTag + ".totOrbAngMomPntN_N", simTimeStep, 0, 2, 'double')
    scSim.AddVariableForLogging(scObject.ModelTag + ".totRotAngMomPntC_N", simTimeStep, 0, 2, 'double')

    #
    #   configure a simulation stop time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # retrieve the logged data for the hub
    r_BN_N = np.array(scStateLog.r_BN_N)
    v_BN_N = np.array(scStateLog.v_BN_N)
    sigma_BN = np.array(scStateLog.sigma_BN)

    # retrieve the logged data for the arm
    timeData = armStateLog.times() * macros.NANO2MIN
    armTheta = np.array(armStateLog.theta)
    armThetaDot = np.array(armStateLog.thetaDot)
    r_ScN_N = np.array(armInertialStateLog.r_BN_N) # r_ScN_N
    v_ScN_N = np.array(armInertialStateLog.v_BN_N) # v_ScN_N
    sigma_SN = np.array(armInertialStateLog.sigma_BN) # sigma_SN
    armInertialAttRate = np.array(armInertialStateLog.omega_BN_B) # omega_SN_S

    # retrieve the logged data for the thruster
    thrusterPosition = []
    for i in range(numTh):
        thrusterPosition.append(np.array(thrStateLog[i].thrusterLocation)) # r_FcS_S

    # Extract the energy and momentum
    orbEnergy = scSim.GetLogVariableData(scObject.ModelTag + ".totOrbEnergy")
    orbAngMom_N = scSim.GetLogVariableData(scObject.ModelTag + ".totOrbAngMomPntN_N")
    rotAngMom_N = scSim.GetLogVariableData(scObject.ModelTag + ".totRotAngMomPntC_N")
    rotEnergy = scSim.GetLogVariableData(scObject.ModelTag + ".totRotEnergy")


    np.set_printoptions(precision=16)

    # difference the thruster positions vs. the arm positions
    thrusterPosDiff = np.zeros([len(timeData),3])
    for i2 in range(len(timeData)):
        # first get the body relative body frame arm position
        r_ScB_N = r_ScN_N[i2] - r_BN_N[i2]
        dcm_BN = RigidBodyKinematics.MRP2C(sigma_BN[i2])
        r_ScN_N_tranposed = np.transpose(r_ScB_N)
        r_ScB_B = dcm_BN.dot(r_ScB_N)
        thrusterPosDiff[i2,:] = r_ScB_B - thrusterPosition[0][i2]

    #
    #   plot the results
    #
    plt.close("all")  # clears out plots from earlier test runs

    plot_armTheta(timeData, armTheta, numTh)
    figureList = {}
    pltName = fileName + "theta history"
    figureList[pltName] = plt.figure(1)

    plot_armThetaDot(timeData, armThetaDot, numTh)
    pltName = fileName + "thetaDot history"
    figureList[pltName] = plt.figure(2)

    plot_thrPosDiff(timeData, thrusterPosDiff)
    pltName = fileName + "thruster position difference"
    figureList[pltName] = plt.figure(3)

    plt.figure()
    plt.clf()
    plt.plot(orbAngMom_N[:, 0] * 1e-9, (orbAngMom_N[:, 1] - orbAngMom_N[0, 1]) / orbAngMom_N[0, 1],
             orbAngMom_N[:, 0] * 1e-9, (orbAngMom_N[:, 2] - orbAngMom_N[0, 2]) / orbAngMom_N[0, 2],
             orbAngMom_N[:, 0] * 1e-9, (orbAngMom_N[:, 3] - orbAngMom_N[0, 3]) / orbAngMom_N[0, 3])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Orbital Angular Momentum')

    plt.figure()
    plt.clf()
    plt.plot(orbEnergy[:, 0] * 1e-9, (orbEnergy[:, 1] - orbEnergy[0, 1]) / orbEnergy[0, 1])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Orbital Energy')

    plt.figure()
    plt.clf()
    plt.plot(rotAngMom_N[:, 0] * 1e-9, (rotAngMom_N[:, 1] - rotAngMom_N[0, 1]) / rotAngMom_N[0, 1],
             rotAngMom_N[:, 0] * 1e-9, (rotAngMom_N[:, 2] - rotAngMom_N[0, 2]) / rotAngMom_N[0, 2],
             rotAngMom_N[:, 0] * 1e-9, (rotAngMom_N[:, 3] - rotAngMom_N[0, 3]) / rotAngMom_N[0, 3])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Rotational Angular Momentum')

    plt.figure()
    plt.clf()
    plt.plot(rotEnergy[:, 0] * 1e-9, (rotEnergy[:, 1] - rotEnergy[0, 1]) / rotEnergy[0, 1])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Rotational Energy')

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    return figureList

#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(
        True,  # show_plots
        False,  # useDVThrusters
    )
