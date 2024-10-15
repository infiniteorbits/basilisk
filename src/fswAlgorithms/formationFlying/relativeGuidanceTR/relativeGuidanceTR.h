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

#ifndef RELATIVE_GUIDANCE_TR_H
#define RELATIVE_GUIDANCE_TR_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"

#include "architecture/msgPayloadDefC/RelNavTransMsgPayload.h"
#include "architecture/msgPayloadDefC/NavAttMsgPayload.h"
#include "architecture/msgPayloadDefC/NavTransMsgPayload.h"
#include "architecture/msgPayloadDefC/CmdForceBodyMsgPayload.h"

/*! @brief basic Basilisk C++ module class */
class RelativeGuidanceTR:public SysModel {
public:
    RelativeGuidanceTR();
    ~RelativeGuidanceTR();

    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);
    void BuildJerkMotion(double d);
    void ComputeJerkMotion(double t, double dva[3]);
    void ReadInputMessages();
    void RotateRelTransToHillClient();
    void ComputeForceOutput();

public:

    double jerk, a_max_in, v_max_in;                       //!< [units] sample module variable declaration
    double waypoint_0_r_BcBs_Hc[3];
    double waypoint_1_r_BcBs_Hc[3];                    //!< [units] sample vector variable
    double direction[3];
    double dva[3];
    double target_r_BcBs_Hc[3];
    double target_v_BcBs_Hc[3];
    double distance, dt_j, dt_a, dt_v, v_max, a_max;
    BSKLogger bskLogger;              //!< -- BSK Logging

    ReadFunctor<RelNavTransMsgPayload> relTransInMsg; // Input message from relative navigation.
    ReadFunctor<NavAttMsgPayload> attInMsg;        //!< attitude navigation output msg
    ReadFunctor<NavTransMsgPayload> transInMsg;    //!< translation navigation output msg

    RelNavTransMsgPayload RelTransState;    //!< relative navigation traslation
    NavAttMsgPayload AttState;
    NavTransMsgPayload TransState;

    double dcm_HcN[3][3]; // to state
    double dcm_HcBs[3][3]; // to state
    double r_BsBc_Hc[3]; // to state
    double v_BsBc_Hc[3]; // to state

    double lqr_gains[3][6];
    double error_BsBc_Hc[6];
    double force_out_Bs[3];

    Message<CmdForceBodyMsgPayload> ForceBodyMsg;

private:
 
    uint64_t t0;
    uint64_t t_prev;
    void sum_jerk(double dva[3], double jerk, double dt);
};


#endif
