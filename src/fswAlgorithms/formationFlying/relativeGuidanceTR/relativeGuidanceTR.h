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
#include "architecture/msgPayloadDefC/CModuleTemplateMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"

/*! @brief basic Basilisk C++ module class */
class RelativeGuidanceTR: public SysModel {
public:
    RelativeGuidanceTR();
    ~RelativeGuidanceTR();

    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);
    void BuildJerkMotion();
    void ComputeJerkMotion();

public:

    double jerk, a_max_in, v_max_in;                       //!< [units] sample module variable declaration
    double waypoint0_RTN[3];
    double waypoint1_RTN[3];                            //!< [units] sample vector variable

    Message<CModuleTemplateMsgPayload> dataOutMsg;     //!< attitude navigation output msg
    ReadFunctor<CModuleTemplateMsgPayload> dataInMsg;  //!< translation navigation output msg

    BSKLogger bskLogger;              //!< -- BSK Logging

private:
    double dt_j, dt_a, dt_v, v_max, a_max;
    double d;
    uint64_t t0;

    void sum_jerk(double* d, double* v, double* a, double jerk, double dt);
};


#endif
