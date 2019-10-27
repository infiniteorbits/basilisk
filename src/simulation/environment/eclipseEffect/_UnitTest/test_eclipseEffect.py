''' '''
'''
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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


import pytest

from Basilisk.simulation import eclipseEffect
from Basilisk.simulation.simMessages import EclipseSimMsg
from Basilisk.simulation.simMessages import SolarFluxSimMsg
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import SimulationBaseClass

@pytest.mark.parametrize("rawFlux, shadowFactor, relTol", [(2.0, 0.5, 1e-8)])
def test_eclipseEffect(show_plots, rawFlux, shadowFactor, relTol):
    """
    Test Description
    -----------------
    This test checks whether the EclipseEffect appropriately modifies solar flux based on eclipse conditions.

    Test Variables
    ---------------
    - rawFlux : [float] greater than or equal to zero
        The rawFlux value is used as the SolarFluxSimMsg input flux value
    - shadowFactor : [float] between 0 and 1
        The shadow factor is used as the EclipseSimMsg shadowFactor and is used to modify the rawFlux
    - relTol : [float] positive
        relative tolerance to which the result is checked

    """
    sim = SimulationBaseClass.SimBaseClass()
    sim.terminateSimulation()
    proc = sim.CreateNewProcess("proc")
    task = sim.CreateNewTask("task", int(1e9))
    proc.addTask(task)

    fluxMsgIn = SolarFluxSimMsg()
    fluxMsgIn.flux = rawFlux
    unitTestSupport.setMessage(sim.TotalSim, proc.Name, "solar_flux", fluxMsgIn, "SolarFluxSimMsg")

    eclipseMsgIn = EclipseSimMsg()
    eclipseMsgIn.shadowFactor = shadowFactor
    unitTestSupport.setMessage(sim.TotalSim, proc.Name, "eclipse_data_0", eclipseMsgIn, "EclipseSimMsg")

    eff = eclipseEffect.EclipseEffect()
    sim.AddModelToTask(task.Name, eff)
    sim.TotalSim.logThisMessage("solar_flux_with_eclipse")
    sim.InitializeSimulationAndDiscover()
    sim.TotalSim.SingleStepProcesses()

    fluxOut = sim.pullMessageLogData("solar_flux_with_eclipse.flux")
    assert fluxOut[0][1] == pytest.approx(fluxMsgIn.flux * eclipseMsgIn.shadowFactor, rel=relTol)
    return


if __name__ == "__main__":
    test_eclipseEffect(False, 2.0, 0.5)