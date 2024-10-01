/*
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

 */
#include "relativeGuidanceTR.h"
#include <iostream>
#include <cstring>
#include <math.h>
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/orbitalMotion.h"

#include "architecture/msgPayloadDefC/RelNavTransMsgPayload.h"
#include "architecture/msgPayloadDefC/NavAttMsgPayload.h"
#include "architecture/msgPayloadDefC/NavTransMsgPayload.h"

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
RelativeGuidanceTR::RelativeGuidanceTR()
{
}

/*! Module Destructor.  */
RelativeGuidanceTR::~RelativeGuidanceTR()
{
    return;
}

void RelativeGuidanceTR::BuildJerkMotion(double d){
    double a_max, v_max, dt_j, dt_a, dt_v, v0, v1;
    a_max = this->a_max_in;
    v_max = this->v_max_in;
    dt_j = a_max / this->jerk;
    dt_a = v_max / a_max - dt_j;
    if (dt_a < 0){
        dt_a = 0;
        a_max = sqrt(v_max*this->jerk);
        dt_j = a_max / this->jerk;
    }
    v0 = 0.5 * this->jerk * dt_j*dt_j;
    v1 = v0 + a_max * dt_a;
    dt_v = (d - (v0 * dt_a + v1*dt_j + v_max*dt_j + v1 * dt_a + v0*dt_j)) / v_max;
    if (dt_v < 0){
        dt_v = 0;
        v_max = (d - v0 * (dt_a + dt_j) - v1 * (dt_a + dt_j)) / dt_j;
    }
    this->v_max = v_max;
    this->a_max = a_max;
    this->dt_j = dt_j;
    this->dt_a = dt_a;
    this->dt_v = dt_v;
}

void RelativeGuidanceTR::sum_jerk(double dva[3], double j, double dt){
    dva[0] += (1.0/6.0)*j*dt*dt*dt + (1.0/2.0)*dva[2]*dt*dt + dva[1]*dt;
    dva[1] += (1.0/2.0)*j*dt*dt + dva[2]*dt;
    dva[2] += j*dt;
}

void RelativeGuidanceTR::ComputeJerkMotion(double t, double dva[3]){
    double t0 = 0;
    double dt;
    bool return_flag = false;
    bskLogger.bskLog(BSK_DEBUG, "t: %f",t);
    for (int i=0; i<3; i++) dva[i] = 0;

    // positive jerk
    if (t <= (t0+this->dt_j)){
        dt = t-t0;
        bskLogger.bskLog(BSK_DEBUG, "positive jerk 1 (t0: %f, dt: %f)",t0, dt);
        return_flag = true;
    } else {
        dt = this->dt_j;
    }
    sum_jerk(dva, this->jerk, dt);
    if (return_flag) return;
    t0 += dt;

    // no jerk
    if (t <= (t0+this->dt_a)){
        dt = t - t0;
        bskLogger.bskLog(BSK_DEBUG, "no jerk 1 (t0: %f, dt: %f)",t0, dt);
        return_flag = true;
    } else {
         dt = this->dt_a;
    }
    sum_jerk(dva, 0, dt);
    if (return_flag) return;
    t0 += dt;
    
    // negative jerk
    if (t <= (t0+this->dt_j)){
        dt = t -t0;
        bskLogger.bskLog(BSK_DEBUG, "negative jerk 1 (t0: %f, dt: %f)",t0, dt);
        return_flag = true;
    } else {
        dt = this->dt_j;
    }
    sum_jerk(dva, -this->jerk, dt);
    if (return_flag) return;
    t0 += dt;

    // constant speed
    if (t <= (t0+this->dt_v)){
        dt = t - t0;
        bskLogger.bskLog(BSK_DEBUG, "constant speed (t0: %f, dt: %f)",t0, dt);
        return_flag = true;
    } else {
        dt = this->dt_v;
    }
    sum_jerk(dva, 0, dt);
    if (return_flag) return;
    t0 += dt;

    // negative jerk
    if (t <= (t0+this->dt_j)){
        dt = t - t0;
        bskLogger.bskLog(BSK_DEBUG, "negative jerk 2 (t0: %f, dt: %f)",t0, dt);
        return_flag = true;
    } else {
        dt = this->dt_j;
    }
    sum_jerk(dva, -this->jerk, dt);
    if (return_flag) return;
    t0 += dt;

    // constant acceleration
    if (t <= (t0+this->dt_a)){
        dt = t - t0;
        bskLogger.bskLog(BSK_DEBUG, "no jerk 2 (t0: %f, dt: %f)",t0, dt);;
        return_flag = true;
    } else {
        dt = this->dt_a;
    }
    sum_jerk(dva, 0, dt);
    if (return_flag) return;
    t0 += dt;

    // positive jerk
    if (t <= (t0+this->dt_j)){
        dt = t - t0;
        bskLogger.bskLog(BSK_DEBUG, "positive jerk 2 (t0: %f, dt: %f)",t0, dt);
        return_flag = true;
    } else {
        dt = this->dt_j;
    }
    sum_jerk(dva, this->jerk, dt);
    if (return_flag) return;

    bskLogger.bskLog(BSK_DEBUG, "no update");
    return;
}

/*! This method reads the input messages associated with the relative navigation
    and the absolute attitude and navigation.
    @return void
 */
void RelativeGuidanceTR::ReadInputMessages(){
    this->RelTransState = this->relTransInMsg();
    this->AttState = this->attInMsg();
    this->TransState = this->transInMsg();
    bskLogger.bskLog(BSK_DEBUG, "AttState.sigma_BN: [%.12f, %.12f, %.12f]", this->AttState.sigma_BN[0],
        this->AttState.sigma_BN[1], this->AttState.sigma_BN[2]);
    bskLogger.bskLog(BSK_DEBUG, "RelTransState.r_BcBs_Bs: [%.12f, %.12f, %.12f]", RelTransState.r_BcBs_Bs[0],
        RelTransState.r_BcBs_Bs[1], RelTransState.r_BcBs_Bs[2]);
    bskLogger.bskLog(BSK_DEBUG, "RelTransState.v_BcBs_Bs: [%.12f, %.12f, %.12f]", RelTransState.v_BcBs_Bs[0],
        RelTransState.v_BcBs_Bs[1], RelTransState.v_BcBs_Bs[2]);
    bskLogger.bskLog(BSK_DEBUG, "TransState.r_BN_N: [%.12f, %.12f, %.12f]", TransState.r_BN_N[0],
        TransState.r_BN_N[1], TransState.r_BN_N[2]);
    bskLogger.bskLog(BSK_DEBUG, "TransState.v_BN_N: [%.12f, %.12f, %.12f]", TransState.v_BN_N[0],
        TransState.v_BN_N[1], TransState.v_BN_N[2]);
}

void RelativeGuidanceTR::RotateRelTransToHillClient(){
    double dcm_BsN[3][3];
    double r_BcBs_N[3];
    double r_BcN_N[3];
    double v_BcBs_N[3];
    double v_BcN_N[3];

    // attState.sigma_BN to DCM_BsN
    MRP2C(this->AttState.sigma_BN, dcm_BsN);
    // rotate r_BcBs_Bs to N and add transState.r_BN_N to obtain client's r_BN_N
    m33MultV3(dcm_BsN, this->RelTransState.r_BcBs_Bs, r_BcBs_N);
    v3Add(r_BcBs_N, this->TransState.r_BN_N, r_BcN_N);
    // rotate v_BcBs_Bs to N, and add transState.v_BN_N to obtain client's v_BN_N
    m33MultV3(dcm_BsN, this->RelTransState.v_BcBs_Bs, v_BcBs_N);
    v3Add(v_BcBs_N, this->TransState.v_BN_N, v_BcN_N);
    // build clients hill frame with hillFrame()
    hillFrame(r_BcN_N, v_BcN_N, this->dcm_HcN);
    bskLogger.bskLog(BSK_DEBUG, "dcm_HcN: [%.12f, %.12f, %.12f]", dcm_HcN[0][0],
        dcm_HcN[1][1], dcm_HcN[2][2]);
    // use DCM_BsN and hill to obtain RTN2Bs rotation
    m33MultM33t(this->dcm_HcN, dcm_BsN, this->dcm_HcBs);
    // rotate target_position_RTN and target_velocity_RTN to Bs
    m33tMultV3(this->dcm_HcBs, this->RelTransState.r_BcBs_Bs, this->r_BcBs_Hc);
    m33tMultV3(this->dcm_HcBs, this->RelTransState.v_BcBs_Bs, this->v_BcBs_Hc);
}

void RelativeGuidanceTR::ComputeForceOutput(){
    double force_out_RTN[3] = {0, 0, 0};
    int i,j;
    for (i=0; i<3; i++){
        this->error_vector_RTN[i] = this->target_position_RTN[i] - this->r_BcBs_Hc[i];
        this->error_vector_RTN[i+3] = this->target_velocity_RTN[i] - this->v_BcBs_Hc[i];
    }
    bskLogger.bskLog(BSK_DEBUG, "error_vector_RTN: [%.12f, %.12f, %.12f,%.12f, %.12f, %.12f]", this->error_vector_RTN[0],
        this->error_vector_RTN[1], this->error_vector_RTN[2], this->error_vector_RTN[3],
        this->error_vector_RTN[4], this->error_vector_RTN[5]);
    for (i=0; i<3; i++){
        for (j=0; j<6; j++){
            force_out_RTN[i] -= this->lqr_gains[i][j]*this->error_vector_RTN[j];
        }
    }
    bskLogger.bskLog(BSK_DEBUG, "force_out_RTN: [%.12f, %.12f, %.12f]", force_out_RTN[0],
        force_out_RTN[1], force_out_RTN[2]);
    m33MultV3(this->dcm_HcBs, force_out_RTN, this->force_out_Bs);
}

/*! This method is used to reset the module.
    @return void
 */
void RelativeGuidanceTR::Reset(uint64_t CurrentSimNanos)
{
    /*! - reset any required variables */
    double distance_vec[3];
    v3Subtract(this->waypoint1_RTN, this->waypoint0_RTN, distance_vec);
    this->distance = v3Norm(distance_vec);
    v3Normalize(distance_vec, this->direction);
    
    BuildJerkMotion(this->distance);
    this->t0 = CurrentSimNanos;
    this->t_prev = CurrentSimNanos;
    bskLogger.bskLog(BSK_INFORMATION, "Variable distance set to %f in reset.",this->distance);
    bskLogger.bskLog(BSK_INFORMATION, "Variable dt_jerk set to %f in reset.",this->dt_j);
    bskLogger.bskLog(BSK_INFORMATION, "Variable dt_acc set to %f in reset.",this->dt_a);
    bskLogger.bskLog(BSK_INFORMATION, "Variable dt_vel set to %f in reset.",this->dt_v);
    bskLogger.bskLog(BSK_INFORMATION, "Variable v_max set to %f in reset.",this->a_max);
    bskLogger.bskLog(BSK_INFORMATION, "Variable a_max set to %f in reset.",this->v_max);

    if (!this->relTransInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "RelativeGuidanceTR.relTransInMsg was not linked.");
    }
    if (!this->attInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "RelativeGuidanceTR.attInMsg was not linked.");
    }
    if (!this->transInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "RelativeGuidanceTR.transInMsg was not linked.");
    }
}

/*! This is the main method that gets called every time the module is updated.  Provide an appropriate description.
    @return void
 */
void RelativeGuidanceTR::UpdateState(uint64_t CurrentSimNanos)
{
    CmdForceBodyMsgPayload cmdForceBodyMsgBuffer;
    cmdForceBodyMsgBuffer = this->ForceBodyMsg.zeroMsgPayload;
    // Just advance the target relative position between the two waypoints.
    double t;
    double target_delta_RTN[3];
    t = (CurrentSimNanos - this->t0) * NANO2SEC;
    ComputeJerkMotion(t, this->dva);
    bskLogger.bskLog(BSK_DEBUG, "dva: [%.12f, %.12f, %.12f]", dva[0], dva[1], dva[2]);
    v3Scale(this->dva[0], this->direction, target_delta_RTN);
    v3Add(target_delta_RTN, this->waypoint0_RTN, this->target_position_RTN);
    v3Scale(this->dva[1], this->direction, this->target_velocity_RTN);
    this->t_prev = CurrentSimNanos;

    ReadInputMessages();
    RotateRelTransToHillClient();
    bskLogger.bskLog(BSK_DEBUG, "r_BcBs_Hc: [%.12f, %.12f, %.12f]", r_BcBs_Hc[0],
        r_BcBs_Hc[1], r_BcBs_Hc[2]);
    bskLogger.bskLog(BSK_DEBUG, "v_BcBs_Hc: [%.12f, %.12f, %.12f]", v_BcBs_Hc[0],
        v_BcBs_Hc[1], v_BcBs_Hc[2]);
    ComputeForceOutput();
    bskLogger.bskLog(BSK_DEBUG, "force_out_Bs: [%.12f, %.12f, %.12f]", this->force_out_Bs[0],
        this->force_out_Bs[1], this->force_out_Bs[2]);
    v3Copy(this->force_out_Bs, cmdForceBodyMsgBuffer.forceRequestBody);
    this->ForceBodyMsg.write(&cmdForceBodyMsgBuffer, this->moduleID, CurrentSimNanos);

}