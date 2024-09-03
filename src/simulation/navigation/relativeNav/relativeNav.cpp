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
#include "simulation/navigation/relativeNav/relativeNav.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include <iostream>
#include <cstring>
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/macroDefinitions.h"

/*! This is the constructor for the simple nav model.  It sets default variable
    values and initializes the various parts of the model */
RelativeNav::RelativeNav()
{
    this->crossTrans = false;
    this->crossAtt = false;
    this->prevTime = 0;
    this->estRelAttState = this->relAttOutMsg.zeroMsgPayload;
    this->trueRelAttState = this->relAttOutMsg.zeroMsgPayload;
    this->estRelTransState = this->relTransOutMsg.zeroMsgPayload;
    this->trueRelTransState = this->relTransOutMsg.zeroMsgPayload;
    this->PMatrix.resize(12,12);
    this->PMatrix.fill(0.0);
    this->walkBounds.resize(12);
    this->walkBounds.fill(0.0);
    this->errorModel =  GaussMarkov(12, this->RNGSeed);
}

/*! Destructor.  Nothing here. */
RelativeNav::~RelativeNav()
{
    return;
}


/*! This method is used to reset the module. It
 initializes the various containers used in the model as well as creates the
 output message.  The error states are allocated as follows:
 Total states: 18
     - Position errors [0-2]
     - Velocity errors [3-5]
     - Attitude errors [6-8]
     - Body Rate errors [9-11]
     - Sun Point error [12-14]
     - Accumulated DV errors [15-17]
 @return void
 */
void RelativeNav::Reset(uint64_t CurrentSimNanos)
{
    // check if input messages has not been included
    if (!this->servicerStateInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "RelativeNav.servicerStateInMsg was not linked.");
    }
    if (!this->clientStateInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "RelativeNav.clientStateInMsg was not linked.");
    }
    int64_t numStates = 12;

    //! - Initialize the propagation matrix to default values for use in update
    this->AMatrix.setIdentity(numStates, numStates);
    this->AMatrix(0,3) = this->AMatrix(1,4) = this->AMatrix(2,5) = this->crossTrans ? 1.0 : 0.0;
    this->AMatrix(6,9) = this->AMatrix(7,10) = this->AMatrix(8, 11) = this->crossAtt ? 1.0 : 0.0;

    //! - Alert the user and stop if the noise matrix is the wrong size.  That'd be bad.
    if (this->PMatrix.size() != numStates*numStates) {
        bskLogger.bskLog(BSK_ERROR, "Your process noise matrix (PMatrix) is not 18*18. Size is %ld.  Quitting", this->PMatrix.size());
        return;
    }
    //! - Set the matrices of the lower level error propagation (GaussMarkov)
    this->errorModel.setNoiseMatrix(this->PMatrix);
    this->errorModel.setRNGSeed(this->RNGSeed);
    if (this->walkBounds.size() != numStates) {
        bskLogger.bskLog(BSK_ERROR, "Your walkbounds vector  is not 12 elements. Quitting");
    }
    this->errorModel.setUpperBounds(this->walkBounds);
}


/*! This method reads the input messages associated with the vehicle state and
 the sun state
 */
void RelativeNav::readInputMessages()
{
    this->servicerInertialState = this->servicerStateInMsg();
    this->clientInertialState = this->clientStateInMsg();

}

/*! This method writes the aggregate nav information into the output state message.
 @return void
 @param Clock The clock time associated with the model call
 */
void RelativeNav::writeOutputMessages(uint64_t Clock)
{
    /* time tage the output message */
    this->estRelAttState.timeTag = (double) Clock * NANO2SEC;
    this->estRelTransState.timeTag = (double) Clock * NANO2SEC;
    
    this->relAttOutMsg.write(&this->estRelAttState, this->moduleID, Clock);
    this->relTransOutMsg.write(&this->estRelTransState, this->moduleID, Clock);
}

void RelativeNav::applyErrors()
{
    double relative_distance_m;
    //! - Error are to be scaled by the relative distance between the two spacecrafts
    relative_distance_m = v3Norm(this->trueRelTransState.r_BcBs_Bs);
    this->navErrors *= relative_distance_m;

    v3Add(this->trueRelTransState.r_BcBs_Bs, &(this->navErrors.data()[0]), this->estRelTransState.r_BcBs_Bs);
    v3Add(this->trueRelTransState.v_BcBs_Bs, &(this->navErrors.data()[3]), this->estRelTransState.v_BcBs_Bs);
    
    addMRP(this->trueRelAttState.sigma_BcBs, &(this->navErrors.data()[6]), this->estRelAttState.sigma_BcBs);
    v3Add(this->trueRelAttState.omega_BcBs_Bs, &(this->navErrors.data()[9]), this->estRelAttState.omega_BcBs_Bs);
}


/*! This method uses the input messages as well as the calculated model errors to
 compute what the output navigation state should be.
    @return void
    @param Clock The clock time associated with the model's update call
*/
void RelativeNav::computeTrueOutput(uint64_t Clock)
{   
    double r_BcBs_N[3];
    double v_BcBs_N[3];
    double v_BcBs_Bs[3];
    double v_BcBs_Bs_cross[3];
    double client_dcm_BN[3][3];
    double servicer_dcm_BN[3][3];
    double dcm_BcBs[3][3];
    double client_omega_BN_Bs[3];

    // Compute relative attitude
    MRP2C(this->clientInertialState.sigma_BN, client_dcm_BN);
    MRP2C(this->servicerInertialState.sigma_BN, servicer_dcm_BN);
    m33MultM33t(client_dcm_BN, servicer_dcm_BN, dcm_BcBs);
    C2MRP(dcm_BcBs, this->trueRelAttState.sigma_BcBs);

    // Compute relative position and velocity
    v3Subtract(this->clientInertialState.r_BN_N, this->servicerInertialState.r_BN_N, r_BcBs_N);
    m33tMultV3(servicer_dcm_BN, r_BcBs_N, this->trueRelTransState.r_BcBs_Bs);
    
    v3Subtract(this->clientInertialState.v_BN_N, this->servicerInertialState.v_BN_N, v_BcBs_N);
    m33tMultV3(servicer_dcm_BN, v_BcBs_N, v_BcBs_Bs);
    v3Cross(this->trueRelTransState.r_BcBs_Bs, this->servicerInertialState.omega_BN_B, v_BcBs_Bs_cross);
    v2Add(v_BcBs_Bs, v_BcBs_Bs_cross, this->trueRelTransState.v_BcBs_Bs);


    m33tMultV3(dcm_BcBs, this->clientInertialState.omega_BN_B, client_omega_BN_Bs);
    v3Subtract(client_omega_BN_Bs, this->servicerInertialState.omega_BN_B, this->trueRelAttState.omega_BcBs_Bs);

}

/*! This method sets the propagation matrix and requests new random errors from
 its GaussMarkov model.
 @return void
 @param CurrentSimNanos The clock time associated with the model call
 */
void RelativeNav::computeErrors(uint64_t CurrentSimNanos)
{
    double timeStep;
    Eigen::MatrixXd localProp = this->AMatrix;
    //! - Compute timestep since the last call
    timeStep = (CurrentSimNanos - this->prevTime)*1.0E-9;

    localProp(0,3) *= timeStep; //postion/velocity cross correlation terms
    localProp(1,4) *= timeStep; //postion/velocity cross correlation terms
    localProp(2,5) *= timeStep; //postion/velocity cross correlation terms
    localProp(6,9) *= timeStep; //attitude/attitude rate cross correlation terms
    localProp(7,10) *= timeStep; //attitude/attitude rate cross correlation terms
    localProp(8,11) *= timeStep; //attitude/attitude rate cross correlation terms

    //! - Set the GaussMarkov propagation matrix and compute errors
    this->errorModel.setPropMatrix(localProp);
    this->errorModel.computeNextState();
    this->navErrors = this->errorModel.getCurrentState();
}

/*! This method calls all of the run-time operations for the simple nav model.
    @return void
    @param CurrentSimNanos The clock time associated with the model call
*/
void RelativeNav::UpdateState(uint64_t CurrentSimNanos)
{
    this->readInputMessages();
    this->computeTrueOutput(CurrentSimNanos);
    this->computeErrors(CurrentSimNanos);
    this->applyErrors();
    this->writeOutputMessages(CurrentSimNanos);
    this->prevTime = CurrentSimNanos;
}
