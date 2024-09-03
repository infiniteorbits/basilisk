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

#ifndef RELATIVE_NAV_H
#define RELATIVE_NAV_H

#include <vector>
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/gauss_markov.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/RelNavAttMsgPayload.h"
#include "architecture/msgPayloadDefC/RelNavTransMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include <Eigen/Dense>
#include "architecture/messaging/messaging.h"

/*! @brief simple navigation module class */
class RelativeNav: public SysModel {
public:
    RelativeNav();
    ~RelativeNav();

    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);
    void computeTrueOutput(uint64_t Clock);
    void computeErrors(uint64_t CurrentSimNanos);
    void applyErrors();
    void readInputMessages();
    void writeOutputMessages(uint64_t Clock);

public:
    Eigen::MatrixXd PMatrix;          //!< -- Cholesky-decomposition or matrix square root of the covariance matrix to apply errors with
    Eigen::VectorXd walkBounds;       //!< -- "3-sigma" errors to permit for states
    Eigen::VectorXd navErrors;        //!< -- Current navigation errors applied to truth
    Message<RelNavAttMsgPayload> relAttOutMsg;        //!< attitude navigation output msg
    Message<RelNavTransMsgPayload> relTransOutMsg;    //!< translation navigation output msg
    bool crossTrans;                  //!< -- Have position error depend on velocity
    bool crossAtt;                    //!< -- Have attitude depend on attitude rate
    RelNavAttMsgPayload trueRelAttState;        //!< -- attitude nav state without errors
    RelNavAttMsgPayload estRelAttState;         //!< -- attitude nav state including errors
    RelNavTransMsgPayload trueRelTransState;    //!< -- translation nav state without errors
    RelNavTransMsgPayload estRelTransState;     //!< -- translation nav state including errors
    SCStatesMsgPayload servicerInertialState; //!< -- input inertial state from Star Tracker
    SCStatesMsgPayload clientInertialState; //!< -- input inertial state from Star Tracker

    BSKLogger bskLogger;              //!< -- BSK Logging

    ReadFunctor<SCStatesMsgPayload> servicerStateInMsg;      //!< servicer spacecraft state input msg
    ReadFunctor<SCStatesMsgPayload> clientStateInMsg;      //!< client spacecraft state input msg

private:
    Eigen::MatrixXd AMatrix;           //!< -- The matrix used to propagate the state
    GaussMarkov errorModel;            //!< -- Gauss-markov error states
    uint64_t prevTime;                 //!< -- Previous simulation time observed
};


#endif
