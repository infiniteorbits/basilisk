#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

"""
Overview
--------

This script sets up a 6-DOF spacecraft orbiting Earth. The goal of this tutorial is to demonstrate
how to configure and use the :ref:`MRP_Steering` module with a rate sub-servo system
the new BSK_Sim architecture.

The script is found in the folder ``src/examples/BskSim/scenarios`` and executed by using::

      python3 scenario_AttSteering.py

The simulation mimics the basic simulation simulation in the earlier tutorial in
:ref:`scenarioAttitudeSteering`.

The simulation layout is shown in the following illustration.

.. image:: /_images/static/test_scenario_AttSteering.svg
   :align: center

The initial conditions for the scenario are the same as found within :ref:`scenario_FeedbackRW`.

Custom Dynamics Configurations Instructions
-------------------------------------------

The dynamics setup is the same as in :ref:`scenario_FeedbackRW`.

Custom FSW Configurations Instructions
--------------------------------------

To configure the desired "steeringRW" FSW mode the user must add the following modules to :ref:`BSK_FSW.py <BSK_FSW>`.



Illustration of Simulation Results
----------------------------------

::

    showPlots = True

.. image:: /_images/Scenarios/scenario_AttSteering_attitudeErrorNorm.svg
   :align: center

.. image:: /_images/Scenarios/scenario_AttSteering_rwMotorTorque.svg
   :align: center

.. image:: /_images/Scenarios/scenario_AttSteering_rateError.svg
   :align: center

.. image:: /_images/Scenarios/scenario_AttSteering_rwSpeed.svg
   :align: center

"""


# Import utilities
from Basilisk.utilities import orbitalMotion, macros, unitTestSupport, vizSupport

# Get current file path
import sys, os, inspect
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# Import master classes: simulation base class and scenario base class
sys.path.append(path + '/..')
from BSK_masters import BSKSim, BSKScenario
import BSK_Dynamics, BSK_Fsw

# Import plotting file for your scenario
sys.path.append(path + '/../plotting')
import BSK_Plotting as BSK_plt


# Create your own scenario child class
class scenario_AttitudeSteeringRW(BSKSim, BSKScenario):
    def __init__(self):
        super(scenario_AttitudeSteeringRW, self).__init__()
        self.name = 'scenario_AttitudeSteeringRW'

        self.set_DynModel(BSK_Dynamics)
        self.set_FswModel(BSK_Fsw)
        self.initInterfaces()

        self.configure_initial_conditions()
        self.log_outputs()

        # if this scenario is to interface with the BSK Viz, uncomment the following line
        # vizSupport.enableUnityVisualization(self, self.DynModels.taskName, self.DynamicsProcessName,
        #                                     gravBodies=self.DynModels.gravFactory,
        #                                     saveFile=filename,
        #                                     numRW=self.DynModels.rwFactory.getNumOfDevices())

    def configure_initial_conditions(self):
        print('%s: configure_initial_conditions' % self.name)
        # Configure FSW mode
        self.modeRequest = 'steeringRW'

        # Configure Dynamics initial conditions
        oe = orbitalMotion.ClassicElements()
        oe.a = 10000000.0  # [m]
        oe.e = 0.01
        oe.i = 33.3 * macros.D2R
        oe.Omega = 48.2 * macros.D2R
        oe.omega = 347.8 * macros.D2R
        oe.f = 85.3 * macros.D2R
        mu = self.get_DynModel().gravFactory.gravBodies['earth'].mu
        rN, vN = orbitalMotion.elem2rv(mu, oe)
        orbitalMotion.rv2elem(mu, rN, vN)
        self.get_DynModel().scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # [m]
        self.get_DynModel().scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # [m/s]
        self.get_DynModel().scObject.hub.sigma_BNInit = [[0.5], [0.6], [-0.3]]
        self.get_DynModel().scObject.hub.omega_BN_BInit = [[0.01], [-0.01], [-0.01]]

    def log_outputs(self):
        print('%s: log_outputs' % self.name)
        samplingTime = self.get_DynModel().processTasksTimeStep
        # Dynamics process outputs:
        self.TotalSim.logThisMessage(self.get_DynModel().rwStateEffector.OutputDataString, samplingTime)
        # FSW process outputs
        samplingTime = self.get_FswModel().processTasksTimeStep
        self.TotalSim.logThisMessage(self.get_FswModel().trackingErrorData.outputDataName, samplingTime)
        self.TotalSim.logThisMessage(self.get_FswModel().mrpSteeringData.outputDataName, samplingTime)
        self.TotalSim.logThisMessage(self.get_FswModel().rwMotorTorqueData.outputDataName, samplingTime)
        return

    def pull_outputs(self, showPlots):
        print('%s: pull_outputs' % self.name)
        num_RW = 4 # number of wheels used in the scenario

        # Dynamics process outputs: pull log messages below if any
        RW_speeds = self.pullMessageLogData( # dataOmegaRW
            self.get_DynModel().rwStateEffector.OutputDataString + ".wheelSpeeds", list(range(num_RW)))

        # FSW process outputs
        dataUsReq = self.pullMessageLogData(
            self.get_FswModel().rwMotorTorqueData.outputDataName + ".motorTorque", list(range(num_RW)))
        sigma_BR = self.pullMessageLogData(
            self.get_FswModel().trackingErrorData.outputDataName + ".sigma_BR", list(range(3)))
        omega_BR_B = self.pullMessageLogData(
            self.get_FswModel().trackingErrorData.outputDataName + ".omega_BR_B", list(range(3)))
        omega_BR_ast = self.pullMessageLogData(
            self.get_FswModel().mrpSteeringData.outputDataName + ".omega_BastR_B", list(range(3)))

        # Plot results
        BSK_plt.clear_all_plots()
        timeData = dataUsReq[:, 0] * macros.NANO2MIN
        BSK_plt.plot_attitude_error(timeData, sigma_BR)
        BSK_plt.plot_rw_cmd_torque(timeData, dataUsReq, num_RW)
        BSK_plt.plot_rate_error(timeData, omega_BR_B)
        BSK_plt.plot_rw_speeds(timeData, RW_speeds, num_RW)
        figureList = {}
        if showPlots:
            BSK_plt.show_all_plots()
        else:
            fileName = os.path.basename(os.path.splitext(__file__)[0])
            figureNames = ["attitudeErrorNorm", "rwMotorTorque", "rateError", "rwSpeed"]
            figureList = BSK_plt.save_all_plots(fileName, figureNames)

        return figureList

def runScenario(scenario):

    # Initialize simulation
    scenario.InitializeSimulationAndDiscover()

    # Configure run time and execute simulation
    simulationTime = macros.min2nano(10.)
    scenario.ConfigureStopTime(simulationTime)
    print('Starting Execution')
    scenario.ExecuteSimulation()
    print('Finished Execution. Post-processing results')

def run(showPlots):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        showPlots (bool): Determines if the script should display plots

    """

    # Configure a scenario in the base simulation
    TheScenario = scenario_AttitudeSteeringRW()
    runScenario(TheScenario)
    figureList = TheScenario.pull_outputs(showPlots)

    return figureList

if __name__ == "__main__":
    run(True)