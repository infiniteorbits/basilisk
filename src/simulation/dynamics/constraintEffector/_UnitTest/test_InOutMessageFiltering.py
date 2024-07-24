# ISC License
#
# Copyright (c) 2024, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

#
#   Unit Test Script
#   Module Name:        constraintEffector
#   Author:             Ananya Kodukula
#   Creation Date:      July 19, 2024
#

import inspect
import os
import pytest
import matplotlib.pyplot as plt
import numpy as np

from Basilisk.utilities import SimulationBaseClass, unitTestSupport, orbitalMotion, macros, RigidBodyKinematics
from Basilisk.simulation import spacecraft, constraintDynamicEffector, gravityEffector, svIntegrators
from Basilisk.architecture import messaging

# uncomment this line if this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail()

@pytest.mark.parametrize("function", ["constraintEffectorInOutMessageFiltering"])
def test_constraintEffector(show_plots, function):
    eval(function + '(show_plots)')

def constraintEffectorInOutMessageFiltering(show_plots):

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)
    testProcessRate = macros.sec2nano(0.001)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create both spacecraft
    scObject1 = spacecraft.Spacecraft()
    scObject1.ModelTag = "spacecraftBody1"
    scObject2 = spacecraft.Spacecraft()
    scObject2.ModelTag = "spacecraftBody2"

    # Set the integrator to RKF45
    integratorObject1 = svIntegrators.svIntegratorRKF45(scObject1)
    scObject1.setIntegrator(integratorObject1)
    # Sync dynamics integration across both spacecraft
    scObject1.syncDynamicsIntegration(scObject2)

    # Define mass properties of the rigid hub of both spacecraft
    scObject1.hub.mHub = 750.0
    scObject1.hub.r_BcB_B = [[0.0], [0.0], [1.0]]
    scObject1.hub.IHubPntBc_B = [[600.0, 0.0, 0.0], [0.0, 600.0, 0.0], [0.0, 0.0, 600.0]]
    scObject2.hub.mHub = 750.0
    scObject2.hub.r_BcB_B = [[0.0], [0.0], [1.0]]
    scObject2.hub.IHubPntBc_B = [[600.0, 0.0, 0.0], [0.0, 600.0, 0.0], [0.0, 0.0, 600.0]]

    # With initial attitudes at zero (B1, B2, and N frames all initially aligned)
    r_B2N_N_0 = np.array([1,1,1])
    rDot_B2N_N = np.array([1,1,1])
    dir = r_B2N_N_0/np.linalg.norm(r_B2N_N_0)
    l = 0.1
    COMoffset = 0.1 # distance from COM to where the arm connects to the spacecraft hub, same for both spacecraft [meters]
    r_P1B1_B1 = np.dot(dir,COMoffset)
    r_P2B2_B2 = np.dot(-dir,COMoffset)
    r_P2P1_B1Init = np.dot(dir,l)
    r_B1N_N_0 = r_B2N_N_0 + r_P2B2_B2 - r_P2P1_B1Init - r_P1B1_B1
    rDot_B1N_N = rDot_B2N_N

    # Compute rotational states
    # let C be the frame at the combined COM of the two vehicles
    r_CN_N = (r_B1N_N_0 * scObject1.hub.mHub + r_B2N_N_0 * scObject2.hub.mHub) / (scObject1.hub.mHub + scObject2.hub.mHub)
    r_B1C_N = r_B1N_N_0 - r_CN_N
    r_B2C_N = r_B2N_N_0 - r_CN_N
    # compute relative velocity due to spin and COM offset
    target_spin = [0.01,0.01,0.01]
    omega_CN_N = np.array(target_spin)
    omega_B1N_B1_0 = omega_CN_N
    omega_B2N_B2_0 = omega_CN_N
    dv_B1C_N = np.cross(omega_CN_N,r_B1C_N)
    dv_B2C_N = np.cross(omega_CN_N,r_B2C_N)
    rDot_B1N_N_0 = rDot_B1N_N + dv_B1C_N
    rDot_B2N_N_0 = rDot_B2N_N + dv_B2C_N

    # Set the initial values for all spacecraft states
    scObject1.hub.r_CN_NInit = r_B1N_N_0
    scObject1.hub.v_CN_NInit = rDot_B1N_N_0
    scObject1.hub.omega_BN_BInit = omega_B1N_B1_0
    scObject2.hub.r_CN_NInit = r_B2N_N_0
    scObject2.hub.v_CN_NInit = rDot_B2N_N_0
    scObject2.hub.omega_BN_BInit = omega_B2N_B2_0

    alpha = 1E2
    beta = 1E2
    k_d = alpha*alpha
    c_d = 2*beta
    wc = 0.1
    h = 1.
    k = 0.7

    # Create the constraint effector module
    constraintEffector = constraintDynamicEffector.ConstraintDynamicEffector()
    # Set up the constraint effector
    constraintEffector.ModelTag = "constraintEffector"
    constraintEffector.setR_P1B1_B1(r_P1B1_B1)
    constraintEffector.setR_P2B2_B2(r_P2B2_B2)
    constraintEffector.setR_P2P1_B1Init(r_P2P1_B1Init)
    constraintEffector.setAlpha(alpha)
    constraintEffector.setBeta(beta)
    constraintEffector.setFilter_Data(h,wc) #0.09
    
    # Add constraints to both spacecraft
    scObject1.addDynamicEffector(constraintEffector)
    scObject2.addDynamicEffector(constraintEffector)
    
    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, scObject1)
    unitTestSim.AddModelToTask(unitTaskName, scObject2)
    unitTestSim.AddModelToTask(unitTaskName, constraintEffector)

    connectionMsgPayload = messaging.ConstDynEffectorConnMsgPayload()
    connectionMsgPayload.value = int(1)
    connectionMsg = messaging.ConstDynEffectorConnMsg().write(connectionMsgPayload)
    constraintEffector.ConstDynEffectorConnInMsg.subscribeTo(connectionMsg)

    # Record the spacecraft state message
    datLog1 = scObject1.scStateOutMsg.recorder()
    datLog2 = scObject2.scStateOutMsg.recorder()
    dataLog3 = constraintEffector.constraintElements.recorder()
    unitTestSim.AddModelToTask(unitTaskName, datLog1)
    unitTestSim.AddModelToTask(unitTaskName, datLog2)
    unitTestSim.AddModelToTask(unitTaskName, dataLog3)

    # Log energy and momentum variables
    conservationData1 = scObject1.logger(["totRotAngMomPntC_N", "totRotEnergy"])
    conservationData2 = scObject2.logger(["totRotAngMomPntC_N", "totRotEnergy"])
    unitTestSim.AddModelToTask(unitTaskName,conservationData1)
    unitTestSim.AddModelToTask(unitTaskName,conservationData2)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    # Setup and run the simulation
    stopTime = 1
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    # collect the recorded spacecraft states
    constraintTimeData = datLog1.times() * macros.NANO2SEC
    r_B1N_N_hist = datLog1.r_BN_N
    rdot_B1N_N_hist = datLog1.v_BN_N
    sigma_B1N_hist = datLog1.sigma_BN
    omega_B1N_B1_hist = datLog1.omega_BN_B
    r_B2N_N_hist = datLog2.r_BN_N
    rdot_B2N_N_hist = datLog2.v_BN_N
    sigma_B2N_hist = datLog2.sigma_BN
    omega_B2N_B2_hist = datLog2.omega_BN_B

    Fc_N = dataLog3.Fc_N
    L_B1 = dataLog3.L_B1
    L_B2 = dataLog3.L_B2
    psi_N = dataLog3.psi_N
    F_filtered = dataLog3.F_filtered
    T1_filtered = dataLog3.T1_filtered
    T2_filtered = dataLog3.T2_filtered

    # Compute constraint violations
    sigma_B2B1 = np.empty(r_B1N_N_hist.shape)
    check_psi_N = np.empty(r_B1N_N_hist.shape)
    psiPrime_N = np.empty(r_B1N_N_hist.shape)
    check_FcN = np.empty(r_B1N_N_hist.shape)
    check_LB1 = np.empty(r_B1N_N_hist.shape)
    check_LB2 = np.empty(r_B1N_N_hist.shape)
    check_filtered_FcN = np.zeros(r_B1N_N_hist.shape[0])
    check_filtered_LB1 = np.zeros(r_B1N_N_hist.shape[0])
    check_filtered_LB2 = np.zeros(r_B1N_N_hist.shape[0])

    for i in range(r_B1N_N_hist.shape[0]):
        dcm_NB1 = np.transpose(RigidBodyKinematics.MRP2C(sigma_B1N_hist[i,:]))
        dcm_B1N = RigidBodyKinematics.MRP2C(sigma_B1N_hist[i,:])
        dcm_B2N = RigidBodyKinematics.MRP2C(sigma_B2N_hist[i,:])
        dcm_NB2 = np.transpose(RigidBodyKinematics.MRP2C(sigma_B2N_hist[i,:]))
        r_P2P1_N = dcm_NB2@r_P2B2_B2+r_B2N_N_hist[i,:]-dcm_NB1@r_P1B1_B1-r_B1N_N_hist[i,:]
        #sigma_B2B1[i,:] = RigidBodyKinematics.subMRP(sigma_B2N_hist[i,:],sigma_B1N_hist[i,:])
        sigma_B2B1[i,:] = RigidBodyKinematics.C2MRP(dcm_B2N@dcm_NB1)
        rDot_P1B1_B1 = np.cross(omega_B1N_B1_hist[i,:],r_P1B1_B1)
        rDot_P2B2_B2 = np.cross(omega_B2N_B2_hist[i,:],r_P2B2_B2)
        rDot_P1N_N = dcm_NB1@rDot_P1B1_B1+rdot_B1N_N_hist[i,:]
        rDot_P2N_N = dcm_NB2@rDot_P2B2_B2+rdot_B2N_N_hist[i,:]
        rDot_P2P1_N = rDot_P2N_N-rDot_P1N_N
        check_psi_N[i,:] = r_P2P1_N - dcm_NB1@r_P2P1_B1Init
        omega_B1N_N = dcm_NB1@omega_B1N_B1_hist[i,:]
        psiPrime_N[i,:] = rDot_P2P1_N - np.cross(omega_B1N_N,r_P2P1_N)
        check_FcN[i,:] = k_d*check_psi_N[i,:]+c_d*psiPrime_N[i,:]

        omega_B1N_B2 = dcm_B2N@omega_B1N_N
        omega_B2B1_B2 = omega_B2N_B2_hist[i,:]-omega_B1N_B2
        Fc_B1 = dcm_B1N@check_FcN[i,:]
        L_B1_len = np.cross(r_P1B1_B1,Fc_B1)
        Fc_B2 = dcm_B2N@check_FcN[i,:]
        L_B2_len = -np.cross(r_P2B2_B2,Fc_B2)
        dcm_B1B2 = dcm_B1N@dcm_NB2
        L_B2_att = -k_d*sigma_B2B1[i,:]-c_d*0.25*RigidBodyKinematics.BmatMRP(sigma_B2B1[i,:])@omega_B2B1_B2
        L_B1_att = -dcm_B1B2@L_B2_att
        check_LB2[i,:] = L_B2_len+L_B2_att
        check_LB1[i,:] = L_B1_len+L_B1_att

    final_psi_compare = np.linalg.norm(psi_N[-1,:]-check_psi_N[-1,:])
    final_FcN_compare = np.linalg.norm(Fc_N[-1,:]-check_FcN[-1,:])
    final_L_B1_compare = np.linalg.norm(L_B1[-1,:]-check_LB1[-1,:])
    final_L_B2_compare = np.linalg.norm(L_B2[-1,:]-check_LB2[-1,:])

    num_coeffs = np.array([np.power(wc * h, 2), 2 * np.power(wc * h, 2), np.power(wc * h, 2)])
    denom_coeffs = np.array([-4 + 4 * k * h - np.power(wc * h, 2),8 - 2 * np.power(wc * h, 2),4 + 4 * k * h + np.power(wc * h, 2)])

    # Calculations
    a = denom_coeffs[1] / denom_coeffs[2]
    b = denom_coeffs[0] / denom_coeffs[2]
    c = num_coeffs[2] / denom_coeffs[2]
    d = num_coeffs[1] / denom_coeffs[2]
    e = num_coeffs[0] / denom_coeffs[2]

    check_filtered_FcN[0:2] = F_filtered[0:2]
    check_filtered_LB1[0:2] = T1_filtered[0:2]
    check_filtered_LB2[0:2] = T2_filtered[0:2]

    for i in range(2,r_B1N_N_hist.shape[0]):
        check_filtered_FcN[i] = a*check_filtered_FcN[i-1]+b*check_filtered_FcN[i-2]+c*np.linalg.norm(Fc_N[i,:])+d*np.linalg.norm(Fc_N[i-1,:])+e*np.linalg.norm(Fc_N[i-2,:])
        check_filtered_LB1[i] = a*check_filtered_LB1[i-1]+b*check_filtered_LB1[i-2]+c*np.linalg.norm(L_B1[i,:])+d*np.linalg.norm(L_B1[i-1,:])+e*np.linalg.norm(L_B1[i-2,:])
        check_filtered_LB2[i] = a*check_filtered_LB2[i-1]+b*check_filtered_LB2[i-2]+c*np.linalg.norm(L_B2[i,:])+d*np.linalg.norm(L_B2[i-1,:])+e*np.linalg.norm(L_B2[i-2,:])

    final_filtered_FcN_compare = F_filtered[-1]-check_filtered_FcN[-1]
    final_filtered_LB1_compare = T1_filtered[-1]-check_filtered_LB1[-1]
    final_filtered_LB2_compare = T2_filtered[-1]-check_filtered_LB2[-1]

    print(final_filtered_FcN_compare)
    print(final_filtered_LB1_compare)
    print(final_filtered_LB2_compare)
    print(final_psi_compare)
    print(final_FcN_compare)
    print(final_L_B1_compare)
    print(final_L_B2_compare)

    # Plotting
    plt.close("all")
    plt.figure(1)
    plt.clf()
    for i in range(3):
        plt.semilogy(constraintTimeData, np.abs(psi_N[:, i]))
    plt.semilogy(constraintTimeData, np.linalg.norm(psi_N,axis=1))
    plt.legend([r'$\psi_1$',r'$\psi_2$',r'$\psi_3$',r'$\psi$ magnitude'])
    plt.xlabel('time (seconds)')
    plt.ylabel(r'variation from fixed position: $\psi$ (meters)')
    plt.title('Direction Constraint Violation Components')

    plt.figure(2)
    plt.clf()
    for i in range(3):
        plt.semilogy(constraintTimeData, np.abs(4*np.arctan(sigma_B2B1[:, i]) * macros.R2D))
    plt.semilogy(constraintTimeData, np.linalg.norm(4*np.arctan(sigma_B2B1) * macros.R2D,axis=1))
    plt.legend([r'$\phi_1$',r'$\phi_2$',r'$\phi_3$',r'$\phi$ magnitude'])
    plt.xlabel('time (seconds)')
    plt.ylabel(r'relative attitude angle: $\phi$ (deg)')
    plt.title('Attitude Constraint Violation Components')

    plt.figure(3)
    plt.clf()
    for i in range(3):
        plt.semilogy(constraintTimeData, np.abs(Fc_N[:, i]))
    plt.semilogy(constraintTimeData, np.linalg.norm(Fc_N,axis=1))
    plt.legend([r'$FcN_1$',r'$FcN_2$',r'$FcN_3$',r'$FcN$ magnitude'])
    plt.xlabel('time (seconds)')
    plt.ylabel(r'Constraint force: $FcN$ (N)')
    plt.title('Constraint Force')

    plt.figure(4)
    plt.clf()
    #for i in range(3):
    #    plt.semilogy(constraintTimeData, np.abs(F_filtered[:, i]))
    plt.semilogy(constraintTimeData, F_filtered)
    plt.semilogy(constraintTimeData, np.linalg.norm(Fc_N,axis=1))
    plt.legend([r'F_filtered magnitude',r'F_unfiltered magnitude'])
    plt.xlabel('time (seconds)')
    plt.ylabel(r'Force(N)')
    plt.title('Comparison between Filtered and Unifiltered Constraint Force')

    plt.figure(5)
    plt.clf()
    #for i in range(3):
    #    plt.semilogy(constraintTimeData, np.abs(F_filtered[:, i]))
    plt.semilogy(constraintTimeData, T2_filtered)
    plt.semilogy(constraintTimeData, np.linalg.norm(L_B2,axis=1))
    plt.legend([r'T1_filtered magnitude',r'T1_unfiltered magnitude'])
    plt.xlabel('time (seconds)')
    plt.ylabel(r'Torque(N.m)')
    plt.title('Comparison between Filtered and Unifiltered Constraint Torque on s/c 2')

    plt.figure(6)
    plt.clf()
    #for i in range(3):
    #    plt.semilogy(constraintTimeData, np.abs(F_filtered[:, i]))
    plt.semilogy(constraintTimeData, T1_filtered)
    plt.semilogy(constraintTimeData, np.linalg.norm(L_B1,axis=1))
    plt.legend([r'T2_filtered magnitude',r'T2_unfiltered magnitude'])
    plt.xlabel('time (seconds)')
    plt.ylabel(r'Torque(N.m)')
    plt.title('Comparison between Filtered and Unifiltered Constraint Torque on s/c 1')

    plt.figure(8)
    plt.clf()
    for i in range(3):
        plt.semilogy(constraintTimeData, np.abs(psi_N[:, i]))
    plt.semilogy(constraintTimeData, np.linalg.norm(psi_N,axis=1))
    plt.legend([r'$\psi_1$',r'$\psi_2$',r'$\psi_3$',r'$\psi$ magnitude'])
    plt.xlabel('time (seconds)')
    plt.ylabel(r'variation from fixed position: $\psi$ (meters)')
    plt.title('Direction Constraint Violation Components in Inertial frame')

    if show_plots:
        plt.show()
    plt.close("all")

    accuracy = 1E-10
    np.testing.assert_allclose(final_psi_compare,0,atol = accuracy, err_msg = 'direction constraint output message norm is incorrect')
    np.testing.assert_allclose(final_FcN_compare,0,atol = accuracy, err_msg = 'constraint force output message norm is incorrect')
    np.testing.assert_allclose(final_L_B1_compare,0,atol = accuracy, err_msg = 'constraint torque on s/c 1 output message norm is incorrect')
    np.testing.assert_allclose(final_L_B2_compare,0,atol = accuracy, err_msg = 'constraint torque on s/c 2 output message norm is incorrect')
    np.testing.assert_allclose(final_filtered_FcN_compare,0,atol = accuracy, err_msg = 'filtered constraint force output message norm is incorrect')
    np.testing.assert_allclose(final_filtered_LB1_compare,0,atol = accuracy, err_msg = 'filtered constraint torque on s/c 1 output message norm is incorrect')
    np.testing.assert_allclose(final_filtered_LB2_compare,0,atol = accuracy, err_msg = 'filtered constraint torque on s/c 2 output message norm is incorrect')

    # # Testing setup
    # accuracy = 1E-12
    # np.testing.assert_allclose(psi_B1, 0, atol=accuracy,
    #                            err_msg='direction constraint component magnitude exceeded in rotational conservation test')
    # np.testing.assert_allclose(sigma_B2B1, 0, atol=accuracy,
    #                            err_msg='attitude constraint component magnitude exceeded in rotational conservation test')
    # for i in range(3):
    #     np.testing.assert_allclose(rotAngMom1PntCT_N[:,i]+rotAngMom2PntCT_N[:,i], rotAngMom1PntCT_N[0,i]+rotAngMom2PntCT_N[0,i], atol=accuracy,
    #                                err_msg='rotational angular momentum difference component magnitude exceeded')
    # np.testing.assert_allclose(rotEnergy1+rotEnergy2, rotEnergy1[0]+rotEnergy2[0], atol=accuracy,
    #                            err_msg='rotational energy difference magnitude exceeded')
    
if __name__ == "__main__":
    constraintEffectorInOutMessageFiltering(True)