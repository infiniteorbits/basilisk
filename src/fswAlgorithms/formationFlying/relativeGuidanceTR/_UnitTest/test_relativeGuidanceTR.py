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
#   Module Name:        cppModuleTemplateParametrized
#   Author:             (First Name) (Last Name)
#   Creation Date:      Month Day, Year
#

import inspect
import os

import numpy as np
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)







# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.fswAlgorithms import relativeGuidanceTR            # import the module that is to be tested
from Basilisk.utilities import macros
from Basilisk.architecture import messaging                      # import the message definitions
from Basilisk.architecture import bskLogging


def relativeGuidanceTRTestFunction():
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)
    bskLogging.setDefaultLogLevel(bskLogging.BSK_INFORMATION)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(1.0)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))


    # Construct algorithm and associated C++ container
    relGuidanceTR_obj = relativeGuidanceTR.RelativeGuidanceTR()
    relGuidanceTR_obj.ModelTag = "RelativeGuidanceTR"            # update python name of test module

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, relGuidanceTR_obj)

    # Initialize the test module configuration data
    relGuidanceTR_obj.v_max_in = 0.01
    relGuidanceTR_obj.a_max_in = 1/666
    relGuidanceTR_obj.jerk = relGuidanceTR_obj.a_max_in / 1000
    distance = 10
    relGuidanceTR_obj.waypoint0_RTN = np.array([0,0,-12])
    relGuidanceTR_obj.waypoint1_RTN = relGuidanceTR_obj.waypoint0_RTN + np.array([0,0,distance])
    relGuidanceTR_obj.BuildJerkMotion(distance)
    print(f"""
              dt_j ={relGuidanceTR_obj.dt_j}s
              dt_a ={relGuidanceTR_obj.dt_a}s
              dt_v ={relGuidanceTR_obj.dt_v}s
              v_max ={relGuidanceTR_obj.v_max} m/s
              a_max ={relGuidanceTR_obj.a_max} m/s/s
           """)
    t_total = relGuidanceTR_obj.dt_j*4+2*relGuidanceTR_obj.dt_a+relGuidanceTR_obj.dt_v

    variableNames = ["dva", "target_position_RTN", "target_velocity_RTN"]
    moduleLogArray = [relGuidanceTR_obj.logger(variableName) for variableName in variableNames]
    for moduleLog in moduleLogArray:
        unitTestSim.AddModelToTask(unitTaskName, moduleLog)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(t_total+200))        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    # print all logging messages
    relGuidanceTR_obj.bskLogger.printLogLevel()

    # This pulls the BSK module internal varialbe log from the simulation run.
    # Note, this should only be done for debugging as it is a slow process
    output_dict = {}
    for variableName,moduleLog in zip(variableNames, moduleLogArray):
        output_dict[variableName] = unitTestSupport.addTimeColumn(moduleLog.times(), getattr(moduleLog, variableName))

    plot = True
    if plot:
        fig, axes = plt.subplots(3,1)
        times = output_dict["dva"][:,0]*macros.NANO2SEC
        axes[0].plot(times, output_dict["dva"][:,1])
        axes[0].set_ylabel("d")
        axes[1].plot(times, output_dict["dva"][:,2])
        axes[1].set_ylabel("v")
        axes[2].plot(times, output_dict["dva"][:,3])
        axes[2].set_ylabel("a")
        plt.show()
    return 1
#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    relativeGuidanceTRTestFunction()
