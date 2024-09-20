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
#include "fsw/formationFlying/relativeGuidanceTR.h"
#include <iostream>
#include <cstring>
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/linearAlgebra.h"

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

void BuildJerkMotion(double d){
    double a_max, v_max, dt_j, dt_a, dt_v, v0, v1;
    a_max = this->a_max_in;
    v_max = this->v_max_in;
    dt_j = a_max / this->jerk;
    dt_a = v_max / a_max - dt_j;
    if (dt_a < 0){
        dt_a = 0;
        a_max = np.sqrt(v_max*this->jerk);
        dt_j = a_max / this->jerk;
    }
    v0 = 0.5 * this->jerk * dt_j**2
    v1 = v0 + a_max * dt_a
    dt_v = (d - (v0 * dt_a + v1*dt_j + v_max*dt_j + v1 * dt_a + v0*dt_j)) / v_max
    if dt_v < 0:
        dt_v = 0
        v_max = (d - v0 * (dt_a + dt_j) - v1 * (dt_a + dt_j)) / dt_j
    this->v_max = v_max;
    this->a_max = a_max;
    this->dt_j = dt_j;
    this->dt_a = dt_a;
    this->dt_v = dt_v;
}

void sum_jerk(double* d, double* v, double* a, double j, double dt){
    d  = 1/6*j*dt*dt*dt + 1/2*a*dt*dt + v0*dt + d;
    v = 1/2*j*dt*dt + a*dt + v;
    a = j*dt + a;
}

void ComputeJerkMotion(uint64_t CurrentSimNanos, double* d, double* v, double* a){
    double t, dt;
    bool return_flag;
    d = 0; v = 0; a = 0;
    t = (CurrentSimNanos - this->t0) / NANO2SEC;
    // positive jerk
    if t < this->dt_j:
        dt = t;
        return_flag = true;
    else:
        dt = dt_j;
    sum_jerk(&d, &v, &a, this->jerk, dt);
    t0 = t0 + dt
    if (return_flag) return;
    
    // no jerk
    if (t < (t0+dt_a)){
        dt = t - t0;
        return_flag = true;
    } else {
        dt = dt_a;
    }
    sum_jerk(&d, &v, &a, 0, dt);
    t0 = t0 + dt;
    if (return_flag) return;
    
    // negative jerk
    if (t < (t0+dt_j)){
        dt = t - t0;
        return_flag = true;
    } else {
        dt = dt_j;
    }
    sum_jerk(&d, &v, &a, -this->jerk, dt);
    t0 = t0 + dt
    if (return_flag) return;

    // constant speed
    if (t < (t0+dt_v)){
        dt = t - t0;
        return_flag = true;
    } else {
        dt = dt_v;
    }
    sum_jerk(&d, &v, &a, 0, dt);
    t0 = t0 + dt;
    if (return_flag) return;
    
    // negative jerk
    if (t < (t0+dt_j)){
        dt = t - t0;
        return_flag = true;
    } else {
        dt = dt_j;
    }
    sum_jerk(&d, &v, &a, -this->jerk, dt);
    t0 = t0 + dt;
    if (return_flag) return;
    
    // constant acceleration
    if t < (t0+dt_a):
        dt = t - t0
        return_flag = true;
    else:
        dt = dt_a
    d,v,a = sum_jerk(d, v, a, 0, dt)
    t0 = t0 + dt
    if return_flag:
        return d,v,a
    
    ## positive jerk
    if t < ( t0 + dt_j):
        dt = t - t0
        return_flag = True
    else:
        dt = dt_j
    d,v,a = sum_jerk(d, v, a, jerk, dt)
    t0 = t0 + dt
    if return_flag:
        return d,v,a
    
    dt = t - t0
    d,v,a = sum_jerk(d, v, a, 0, dt)
    return d,v,a

}

/*! This method is used to reset the module.
    @return void
 */
void RelativeGuidanceTR::Reset(uint64_t CurrentSimNanos)
{
    /*! - reset any required variables */
    double distance_vec[3];
    double d;
    v3Subtract(this->waypoint1_RTN, this->waypoint0_RTN, distance_vec);
    this->d = v3Norm(this->waypoint0_RTN);
    v3Normalize(distance_vec, this->direction);
    
    BuildJerkMotion(this->d);
    this->t0 = CurrentSimNanos;

    bskLogger.bskLog(BSK_INFORMATION, "Variable distance set to %f in reset.",this->d);
    bskLogger.bskLog(BSK_INFORMATION, "Variable dt_jerk set to %f in reset.",this->dt_j);
    bskLogger.bskLog(BSK_INFORMATION, "Variable dt_acc set to %f in reset.",this->dt_a);
    bskLogger.bskLog(BSK_INFORMATION, "Variable dt_vel set to %f in reset.",this->dt_v);
    bskLogger.bskLog(BSK_INFORMATION, "Variable v_max set to %f in reset.",this->a_max);
    bskLogger.bskLog(BSK_INFORMATION, "Variable a_max set to %f in reset.",this->v_max);

}
void ComputeJerkMotion();

/*! This is the main method that gets called every time the module is updated.  Provide an appropriate description.
    @return void
 */
void RelativeGuidanceTR::UpdateState(uint64_t CurrentSimNanos)
{
    double Lr[3];                                   /*!< [unit] variable description */
    CModuleTemplateMsgPayload outMsgBuffer;       /*!< local output message copy */
    CModuleTemplateMsgPayload inMsgBuffer;        /*!< local copy of input message */
    double  inputVector[3];

    // always zero the output buffer first
    outMsgBuffer = this->dataOutMsg.zeroMsgPayload;
    v3SetZero(inputVector);

    /*! - Read the optional input messages */
    if (this->dataInMsg.isLinked()) {
        inMsgBuffer = this->dataInMsg();
        v3Copy(inMsgBuffer.dataVector, inputVector);
    }

    /*! - Add the module specific code */
    v3Copy(inputVector, Lr);
    this->dummy += 1.0;
    Lr[0] += this->dummy;

    /*! - store the output message */
    v3Copy(Lr, outMsgBuffer.dataVector);

    /*! - write the module output message */
    this->dataOutMsg.write(&outMsgBuffer, this->moduleID, CurrentSimNanos);

    /* this logging statement is not typically required.  It is done here to see in the
     quick-start guide which module is being executed */
    bskLogger.bskLog(BSK_INFORMATION, "C++ Module ID %lld ran Update at %fs", this->moduleID, (double) CurrentSimNanos/(1e9));

}
