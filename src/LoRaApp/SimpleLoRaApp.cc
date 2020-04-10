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

#include "SimpleLoRaApp.h"
#include "inet/mobility/static/StationaryMobility.h"
#include "LoRa/LoRaMacFrame_m.h"

// Senkronize time-->ilkinin atacağı senkronizasyon zamanı
// Time to receive-->Herbirine assign edilecek receive zamanı
// Open receiver -->Time to receivele bağlantılı olarak transceiverı açmak için gönderilen mesaj
// Close receiver -->Open receiverdan sonra kapatmak için gerekli olan mesaj, kapat diyor yani
// Time to next packet, kendine gelen paketleri kaydedip, sıra sıra yollayacak
// Duty Cycle olayı implemente edilecek
// Gerekli time duty cycle için 24.6784 olacak
// ack gelince göndericek
namespace inet {

Define_Module(SimpleLoRaApp);
//REPEATER NODE ACK BEKLEMEYECEK
void SimpleLoRaApp::initialize(int stage)
{
    cSimpleModule::initialize(stage);
    if (stage == INITSTAGE_LOCAL) {
        std::pair<double,double> coordsValues = std::make_pair(-1, -1);
        cModule *host = getContainingNode(this);
        // Generate random location for nodes if circle deployment type
        if (strcmp(host->par("deploymentType").stringValue(), "circle")==0) {
           coordsValues = generateUniformCircleCoordinates(host->par("maxGatewayDistance").doubleValue(), host->par("gatewayX").doubleValue(), host->par("gatewayY").doubleValue());
           StationaryMobility *mobility = check_and_cast<StationaryMobility *>(host->getSubmodule("mobility"));
           mobility->par("initialX").setDoubleValue(coordsValues.first);
           mobility->par("initialY").setDoubleValue(coordsValues.second);
        }
    }
    else if (stage == INITSTAGE_APPLICATION_LAYER) {
        bool isOperational;
        NodeStatus *nodeStatus = dynamic_cast<NodeStatus *>(findContainingNode(this)->getSubmodule("status"));
        isOperational = (!nodeStatus) || nodeStatus->getState() == NodeStatus::UP;
        if (!isOperational)
            throw cRuntimeError("This module doesn't support starting in node DOWN state");
        do {
            timeToFirstPacket = par("timeToFirstPacket");
            EV << "Wylosowalem czas :" << timeToFirstPacket << endl;
            //if(timeToNextPacket < 5) error("Time to next packet must be grater than 3");
        } while(timeToFirstPacket <= 5);
        transmissionQueue.setName("transmissionQueue");
        timeToFirstPacket = par("timeToFirstPacket");
        periodForSending = par("periodForSending");
        timeToReceive = par("timeToReceive");
        openReceiver = new cMessage("openReceiver"); //La bunları handle etmedik ya
        closeReceiver = new cMessage("closeReceiver"); //La bunları handle etmedik ya
        timeOutForAck = new cMessage("timeOutForAck");
        synchronizeTime = new cMessage("timeSynchronization");
        sendLastTime = new cMessage("sendLastTime");
        sendMeasurements = new cMessage("sendMeasurements");
        sendAgain = new cMessage("sendAgain");
        scheduleAt(simTime()+timeToReceive,openReceiver);
        if(!this->getParentModule()->getIndex()){ // 0 Senkronize isteği gönderiyor
            if(macLayer == nullptr){
                macLayer = check_and_cast<LoRaMac *> (getParentModule()->getSubmodule("LoRaNic")->getSubmodule("mac"));
            }
            macLayer->setParentAddress(DevAddr::BROADCAST_ADDRESS);
            synchFlag = true;
            timeToSendGateway = par("timeToSendGateway");
            //sendMeasurements = new cMessage("sendMeasurements");
            scheduleAt(simTime()+timeToSendGateway,sendMeasurements);
            scheduleAt(simTime()+timeToFirstPacket,synchronizeTime); //İlk mesaj
        }

        sentPackets = 0;
        receivedADRCommands = 0;
        numberOfPacketsToSend = par("numberOfPacketsToSend");

        LoRa_AppPacketSent = registerSignal("LoRa_AppPacketSent");

        //LoRa physical layer parameters
        loRaTP = par("initialLoRaTP").doubleValue();
        loRaCF = units::values::Hz(par("initialLoRaCF").doubleValue());
        loRaSF = par("initialLoRaSF");
        loRaBW = inet::units::values::Hz(par("initialLoRaBW").doubleValue());
        loRaCR = par("initialLoRaCR");
        loRaUseHeader = par("initialUseHeader");
        evaluateADRinNode = par("evaluateADRinNode");
        sfVector.setName("SF Vector");
        tpVector.setName("TP Vector");

    }
}

std::pair<double,double> SimpleLoRaApp::generateUniformCircleCoordinates(double radius, double gatewayX, double gatewayY)
{
    double randomValueRadius = uniform(0,(radius*radius));
    double randomTheta = uniform(0,2*M_PI);

    // generate coordinates for circle with origin at 0,0
    double x = sqrt(randomValueRadius) * cos(randomTheta);
    double y = sqrt(randomValueRadius) * sin(randomTheta);
    // Change coordinates based on coordinate system used in OMNeT, with origin at top left
    x = x + gatewayX;
    y = gatewayY - y;
    std::pair<double,double> coordValues = std::make_pair(x,y);
    return coordValues;
}

void SimpleLoRaApp::finish()
{
    cModule *host = getContainingNode(this);
    StationaryMobility *mobility = check_and_cast<StationaryMobility *>(host->getSubmodule("mobility"));
    Coord coord = mobility->getCurrentPosition();
    recordScalar("positionX", coord.x);
    recordScalar("positionY", coord.y);
    recordScalar("finalTP", loRaTP);
    recordScalar("finalSF", loRaSF);
    recordScalar("sentPackets", sentPackets);
    recordScalar("receivedADRCommands", receivedADRCommands);
}

void SimpleLoRaApp::handleMessage(cMessage *msg)
{
    if (msg->isSelfMessage()) {
        if (msg == synchronizeTime){ //Senkronize paketini propagate et
            sendSynchronizeRequest();
            if (simTime() >= getSimulation()->getWarmupPeriod())
               sentPackets++;
        }
        else if(msg == openReceiver){ // Transceiverını aç
            EV_INFO << "NODE " << this->getParentModule()->getIndex() << " OPENS IT'S RECEIVER\n";
            //TELL MAC TO OPEN RADIO
            if(macLayer == nullptr){
                macLayer = check_and_cast<LoRaMac *> (getParentModule()->getSubmodule("LoRaNic")->getSubmodule("mac"));
            }
            macLayer->turnOnReceiver();
            scheduleAt(simTime()+periodForSending,openReceiver);
            scheduleAt(simTime()+9,closeReceiver);
        }
        else if(msg == sendAgain){
            EV << "Periodically send again\n";
            handleMessage(sendMeasurements);
        }
        else if(msg == closeReceiver){ // Transceiverını kapat

            EV_INFO << "NODE " << this->getParentModule()->getIndex() << " CLOSES IT'S RECEIVER\n";
            //LoRaMac *macLayer = check_and_cast<LoRaMac *> (getParentModule()->getSubmodule("LoRaNic")->getSubmodule("mac"));
            //TELL MAC TO CLOSE RADIO
            if(macLayer == nullptr){
                            macLayer = check_and_cast<LoRaMac *> (getParentModule()->getSubmodule("LoRaNic")->getSubmodule("mac"));
            }
            macLayer->turnOffReceiver();
             // Periyodik hale getir
            //scheduleAt(simTime()+periodForSending,closeReceiver); //Periyodik açılış
        }
        else if(msg == timeOutForAck){
            EV << "Timeout, NO ACKNOWLEDGE MESSAGE HAS REACHED\n";
            handleMessage(sendMeasurements);
        }
        else if (msg == sendMeasurements){ //ack aldıktan sonraya çeviricez
            if(macLayer == nullptr){
                macLayer = check_and_cast<LoRaMac *> (getParentModule()->getSubmodule("LoRaNic")->getSubmodule("mac"));
            }
            if(macLayer->numberOfLeftDataMessages == -1){
                macLayer->numberOfLeftDataMessages=transmissionQueue.getLength()+1;
            }
            if(transmissionQueue.front() != NULL){
                if(firstTime){ //periodic yolla for 0
                    scheduleAt(simTime()+periodForSending,sendAgain); // İkisi karısıyor
                    firstTime=false;
                }
                sendQueuedPackets(transmissionQueue.front()->dup());
                transmissionQueue.pop();
                if (simTime() >= getSimulation()->getWarmupPeriod())
                                            sentPackets++;
                if(this->getParentModule()->getIndex() != 0){ //Tree yapısı için birden fazla node
                    scheduleAt(simTime()+2,timeOutForAck);
                }
                else{
                    scheduleAt(simTime()+2, sendMeasurements); // İkisi karısıyor
                }
            }
            else{
                sendJoinRequest();
                if (simTime() >= getSimulation()->getWarmupPeriod())
                                sentPackets++;
                if(getParentModule()->getIndex() != 0)
                if(firstTime){
                    scheduleAt(simTime()+periodForSending,sendAgain); // Means leaf node
                }
                firstTime=true;
            }
        }
    }
    else
    {
        handleMessageFromLowerLayer(msg);
        //delete msg; TODO Will be deleted
    }
}

void SimpleLoRaApp::sendSynchronizeRequest(){
     LoRaAppPacket *request = new LoRaAppPacket("SynchronizePacket");
     request->setKind(1);
     request->setMsgType(JOIN_REQUEST);
     request->setNextPacketTime(timeToReceive);
     request->setCurrentTime(simTime());
     request->setSenderAddr(macLayer->getAddress());
     //add LoRa control info
     LoRaMacControlInfo *cInfo = new LoRaMacControlInfo;
     cInfo->setLoRaTP(loRaTP);
     cInfo->setLoRaCF(loRaCF);
     cInfo->setLoRaSF(loRaSF);
     cInfo->setLoRaBW(loRaBW);
     cInfo->setLoRaCR(loRaCR);

     request->setControlInfo(cInfo);
     //sfVector.record(loRaSF);
     //tpVector.record(loRaTP);
     send(request, "appOut");
}

void SimpleLoRaApp::handleMessageFromLowerLayer(cMessage *msg)
{
    LoRaAppPacket *packet = check_and_cast<LoRaAppPacket *>(msg);
    if(packet->getMsgType() == JOIN_REQUEST){ //Synch alan kapatsın kendini
        if(!synchFlag){
            EV << "RECEIVED SYNCHRONIZATION REQUEST: Sender Time: " << packet->getNextPacketTime()<< " Delay is "<< simTime()-packet->getCurrentTime()<<"\n";
            timeToSendGateway = packet->getNextPacketTime()-packet->getCurrentTime()+this->getParentModule()->getIndex();
            //delete msg;
            if(macLayer == nullptr){
                macLayer = check_and_cast<LoRaMac *> (getParentModule()->getSubmodule("LoRaNic")->getSubmodule("mac"));
            }
            macLayer->setParentAddress(packet->getSenderAddr());
            sendSynchronizeRequest();
            if (simTime() >= getSimulation()->getWarmupPeriod())
                            sentPackets++;
            scheduleAt(simTime()+timeToSendGateway,sendMeasurements);
            macLayer->turnOffReceiver();
        }
        else{
            EV << "WE HAVE ALREADY SYNCHED";
            //delete msg;
        }

    }
    else if(packet->getMsgType() == DATA){
        EV << "DATA HAS COME WE NEED TO ADD IT TO QUEUE SEND AN ACK FROM: "<< packet->getSampleMeasurement() << " \n";

        transmissionQueue.insert(packet);
        sendAckRequest(packet->getSenderAddr());
    }
    else if(packet->getMsgType() == ACKNOWLEDGE){
        EV << "ACK HAS CAME\n";
        messageCopyForResend = nullptr;
        cancelEvent(timeOutForAck);
        if(turnOffReceiverFlag){
            if(macLayer == nullptr){
                macLayer = check_and_cast<LoRaMac *> (getParentModule()->getSubmodule("LoRaNic")->getSubmodule("mac"));
            }
            macLayer->turnOffReceiver();
            //macLayer->turnOffReceiver();
            turnOffReceiverFlag = false;
            //scheduleAt(simTime()+timeToNextPacket,sendMeasurements);
        }
        else{
            scheduleAt(simTime()+0.1,sendMeasurements);
        }
        //1-ACKNOWLEDGE GELİNCE MEASUREMENTS YOLLIYCAK
        //2-SON MEASUREMENT GELİNCE TURNOFFRECEİVER KANKA

    }
}

bool SimpleLoRaApp::handleOperationStage(LifecycleOperation *operation, int stage, IDoneCallback *doneCallback)
{
    Enter_Method_Silent();

    throw cRuntimeError("Unsupported lifecycle operation '%s'", operation->getClassName());
    return true;
}

void SimpleLoRaApp::sendJoinRequest()
{
    LoRaAppPacket *request;
    if(messageCopyForResend == nullptr || this->getParentModule()->getIndex() == 0){
        request = new LoRaAppPacket("DataFrame");
    }
    else{
        request = messageCopyForResend;
    }
    request->setKind(DATA);
    lastSentMeasurement = this->getParentModule()->getIndex();//rand();
    request->setSampleMeasurement(lastSentMeasurement);
    request->setMsgType(DATA);
    if(macLayer == nullptr){
        macLayer = check_and_cast<LoRaMac *> (getParentModule()->getSubmodule("LoRaNic")->getSubmodule("mac"));
    }
    request->setSenderAddr(macLayer->getAddress());
    if(evaluateADRinNode && sendNextPacketWithADRACKReq)
    {
        request->getOptions().setADRACKReq(true);
        sendNextPacketWithADRACKReq = false;
    }

    //add LoRa control info
    LoRaMacControlInfo *cInfo = new LoRaMacControlInfo;
    cInfo->setLoRaTP(loRaTP);
    cInfo->setLoRaCF(loRaCF);
    cInfo->setLoRaSF(loRaSF);
    cInfo->setLoRaBW(loRaBW);
    cInfo->setLoRaCR(loRaCR);

    request->setControlInfo(cInfo);
    sfVector.record(loRaSF);
    tpVector.record(loRaTP);
    send(request, "appOut");
    if(messageCopyForResend == nullptr){
        messageCopyForResend = request->dup();
    }
    if(evaluateADRinNode)
    {
        ADR_ACK_CNT++;
        if(ADR_ACK_CNT == ADR_ACK_LIMIT) sendNextPacketWithADRACKReq = true;
        if(ADR_ACK_CNT >= ADR_ACK_LIMIT + ADR_ACK_DELAY)
        {
            ADR_ACK_CNT = 0;
            increaseSFIfPossible();
        }
    }
    emit(LoRa_AppPacketSent, loRaSF);
    turnOffReceiverFlag = true; //Kardes en son gonderirken bunu true yaptim
}

void SimpleLoRaApp::increaseSFIfPossible()
{
    if(loRaSF < 12) loRaSF++;
}

void SimpleLoRaApp::sendQueuedPackets(cMessage *msg){
    //TODO Resend Code will be implemented.
        LoRaAppPacket *request = static_cast<LoRaAppPacket*>(msg);
        /*if(messageCopyForResend == nullptr){
            request = new LoRaAppPacket("QueuedDataFrame");
        }
        else{
            request = messageCopyForResend;
        } */
        //add LoRa control info
        LoRaMacControlInfo *cInfo = new LoRaMacControlInfo;
        cInfo->setLoRaTP(loRaTP);
        cInfo->setLoRaCF(loRaCF);
        cInfo->setLoRaSF(loRaSF);
        cInfo->setLoRaBW(loRaBW);
        cInfo->setLoRaCR(loRaCR);

        request->setControlInfo(cInfo);
        sfVector.record(loRaSF);
        tpVector.record(loRaTP);
        send(request, "appOut");
        /*if(messageCopyForResend == nullptr){
            messageCopyForResend = request->dup();
        } */
        emit(LoRa_AppPacketSent, loRaSF);
}

void SimpleLoRaApp::sendAckRequest(DevAddr receiverAddr){
         LoRaAppPacket *request = new LoRaAppPacket("AcknowledgementPacket");
         request->setKind(1);
         request->setMsgType(ACKNOWLEDGE);
         request->setNextPacketTime(timeToReceive);
         request->setCurrentTime(simTime());
         //add LoRa control info
         LoRaMacControlInfo *cInfo = new LoRaMacControlInfo;
         cInfo->setLoRaTP(loRaTP);
         cInfo->setLoRaCF(loRaCF);
         cInfo->setLoRaSF(loRaSF);
         cInfo->setLoRaBW(loRaBW);
         cInfo->setLoRaCR(loRaCR);
         cInfo->setDest(receiverAddr);
         request->setControlInfo(cInfo);
         //sfVector.record(loRaSF);
         //tpVector.record(loRaTP);

         send(request, "appOut");
 }

} //end namespace inet
