//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#ifndef __LORA_OMNET_SIMPLELORAAPP_H_
#define __LORA_OMNET_SIMPLELORAAPP_H_

#include <omnetpp.h>
#include "inet/common/lifecycle/ILifecycle.h"
#include "inet/common/lifecycle/NodeStatus.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/lifecycle/LifecycleOperation.h"
#include "LoRaAppPacket_m.h"
#include "LoRa/LoRaMacControlInfo_m.h"
#include "misc/DevAddr.h"
#include "LoRa/LoRaMac.h"

using namespace omnetpp;

namespace inet {

/**
 * TODO - Generated class
 */
class INET_API SimpleLoRaApp : public cSimpleModule, public ILifecycle
{
    protected:
        virtual void initialize(int stage) override;
        void finish() override;
        virtual int numInitStages() const override { return NUM_INIT_STAGES; }
        virtual void handleMessage(cMessage *msg) override;
        virtual bool handleOperationStage(LifecycleOperation *operation, int stage, IDoneCallback *doneCallback) override;
        void sendSynchronizeRequest();
        void handleMessageFromLowerLayer(cMessage *msg);
        std::pair<double,double> generateUniformCircleCoordinates(double radius, double gatewayX, double gatewayY);
        void sendJoinRequest();
        void sendAckRequest(DevAddr receiver);
        void sendQueuedPackets(cMessage *msg);
        int synchOffset = 0;
        //void finishTransmission();
        int numberOfPacketsToSend;
        int sentPackets;
        int receivedADRCommands;
        int lastSentMeasurement;
        simtime_t timeToFirstPacket;
        simtime_t timeToSendGateway;
        simtime_t timeToReceive;
        simtime_t periodForSending;
        bool firstTime = true;
        bool synchFlag = false;
        cMessage *sendLastTime;
        cMessage *configureLoRaParameters;
        cMessage *sendMeasurements;
        cMessage *synchronizeTime;
        cMessage *openReceiver;
        cMessage *closeReceiver;
        cMessage *timeOutForAck;
        cMessage *sendAgain;
        LoRaAppPacket *messageCopyForResend = nullptr;
        cPacketQueue transmissionQueue;

        //history of sent packets;
        cOutVector sfVector;
        cOutVector tpVector;
        bool turnOffReceiverFlag = false;
        //variables to control ADR
        bool evaluateADRinNode;
        int ADR_ACK_CNT = 0;
        int ADR_ACK_LIMIT = 64; //64;
        int ADR_ACK_DELAY = 32; //32;
        bool sendNextPacketWithADRACKReq = false;
        void increaseSFIfPossible();
        LoRaMac *macLayer = nullptr;
    public:
        SimpleLoRaApp() {}
        simsignal_t LoRa_AppPacketSent;
        //LoRa physical layer parameters
        double loRaTP;
        units::values::Hz loRaCF;
        int loRaSF;
        units::values::Hz loRaBW;
        int loRaCR;
        bool loRaUseHeader;
};

}

#endif
