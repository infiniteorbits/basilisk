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

/*! @brief basic Basilisk C++ module class */
class RelativeGuidanceTR:public SysModel {
public:
    RelativeGuidanceTR();
    ~RelativeGuidanceTR();

    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);
    void BuildJerkMotion(double d);
    void ComputeJerkMotion(double t, double* dva);

public:

    double jerk, a_max_in, v_max_in;                       //!< [units] sample module variable declaration
    double waypoint0_RTN[3];
    double waypoint1_RTN[3];                            //!< [units] sample vector variable
    double direction[3];
    double dva[3];
    double target_position_RTN[3];
    double target_velocity_RTN[3];
    double distance, dt_j, dt_a, dt_v, v_max, a_max;
    BSKLogger bskLogger;              //!< -- BSK Logging

private:
 
    uint64_t t0;
    uint64_t t_prev;
    void sum_jerk(double* dva, double jerk, double dt);
};


#endif
