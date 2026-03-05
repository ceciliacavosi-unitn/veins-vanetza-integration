//
// Copyright (C) 2011 David Eckhoff <eckhoff@cs.fau.de>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// SPDX-License-Identifier: GPL-2.0-or-later
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"

using namespace veins;

// ============================================================
//  VeinsVehicleDataProvider  —  bridge Veins <-> Vanetza
// ============================================================

VeinsVehicleDataProvider::VeinsVehicleDataProvider(TraCIMobility* mob)
    : mMobility(mob)
{
}

vanetza::units::GeoAngle VeinsVehicleDataProvider::latitude() const
{
    // TraCI restituisce coordinate in metri (proiezione), convertiamo in gradi
    return vanetza::units::GeoAngle {
        mMobility->getCurrentPosition().y * boost::units::degree::degree
    };
}

vanetza::units::GeoAngle VeinsVehicleDataProvider::longitude() const
{
    return vanetza::units::GeoAngle {
        mMobility->getCurrentPosition().x * boost::units::degree::degree
    };
}

vanetza::units::Velocity VeinsVehicleDataProvider::speed() const
{
    return vanetza::units::Velocity {
        mMobility->getCurrentSpeed() * boost::units::si::meter_per_second
    };
}

vanetza::units::Angle VeinsVehicleDataProvider::heading() const
{
    return vanetza::units::Angle {
        mMobility->getCurrentAngle() * boost::units::degree::degree
    };
}

uint32_t VeinsVehicleDataProvider::station_id() const
{
    // Usa l'ID esterno del veicolo TraCI come stationID ETSI
    return static_cast<uint32_t>(std::hash<std::string>{}(mMobility->getExternalId()));
}

vanetza::Clock::time_point VeinsVehicleDataProvider::timestamp() const
{
    // Converti simTime in time_point Vanetza
    using namespace std::chrono;
    auto ms = duration_cast<milliseconds>(duration<double>(simTime().dbl()));
    return vanetza::Clock::time_point(ms);
}

// ============================================================
//  Helpers CAM (Vanetza ETSI EN 302 637-2)
// ============================================================

/**
 * Costruisce un CAM ETSI-compliant a partire dal VehicleDataProvider.
 * Popola: header, BasicContainer, HighFrequencyContainer.
 */
static vanetza::asn1::Cam buildCam(const VeinsVehicleDataProvider& vdp)
{
    vanetza::asn1::Cam cam;

    // --- ITS PDU Header ---
    cam->header.messageID       = ItsPduHeader__messageID_cam;
    cam->header.protocolVersion = 2;
    cam->header.stationID       = vdp.station_id();

    // --- GenerationDeltaTime (modulo 65536 ms) ---
    using namespace std::chrono;
    auto now_ms = duration_cast<milliseconds>(
        vdp.timestamp().time_since_epoch()).count();
    cam->cam.generationDeltaTime =
        static_cast<GenerationDeltaTime_t>(now_ms % 65536);

    // --- BasicContainer ---
    auto& basic = cam->cam.camParameters.basicContainer;
    basic.stationType = StationType_passengerCar;

    // Posizione in decimi di micro-grado (ETSI: 1e-7 gradi)
    basic.referencePosition.latitude =
        static_cast<Latitude_t>(vdp.latitude().value()  * 1e7);
    basic.referencePosition.longitude =
        static_cast<Longitude_t>(vdp.longitude().value() * 1e7);
    basic.referencePosition.positionConfidenceEllipse.semiMajorConfidence =
        SemiAxisLength_unavailable;
    basic.referencePosition.positionConfidenceEllipse.semiMinorConfidence =
        SemiAxisLength_unavailable;
    basic.referencePosition.positionConfidenceEllipse.semiMajorOrientation =
        HeadingValue_unavailable;
    basic.referencePosition.altitude.altitudeValue =
        AltitudeValue_unavailable;
    basic.referencePosition.altitude.altitudeConfidence =
        AltitudeConfidence_unavailable;

    // --- HighFrequencyContainer ---
    auto& hfc = cam->cam.camParameters.highFrequencyContainer;
    hfc.present = HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;
    auto& bvchf = hfc.choice.basicVehicleContainerHighFrequency;

    // Velocità in cm/s (ETSI: 0..16383, 16383 = unavailable)
    bvchf.speed.speedValue =
        static_cast<SpeedValue_t>(vdp.speed().value() * 100.0);
    bvchf.speed.speedConfidence = SpeedConfidence_unavailable;

    // Heading in decimi di grado (ETSI: 0..3601, 3601 = unavailable)
    bvchf.heading.headingValue =
        static_cast<HeadingValue_t>(vdp.heading().value() * 10.0);
    bvchf.heading.headingConfidence = HeadingConfidence_unavailable;

    // Campi obbligatori con valori "unavailable"
    bvchf.driveDirection              = DriveDirection_forward;
    bvchf.vehicleLength.vehicleLengthValue = VehicleLengthValue_unavailable;
    bvchf.vehicleLength.vehicleLengthConfidenceIndication =
        VehicleLengthConfidenceIndication_unavailable;
    bvchf.vehicleWidth                = VehicleWidth_unavailable;
    bvchf.longitudinalAcceleration.longitudinalAccelerationValue =
        LongitudinalAccelerationValue_unavailable;
    bvchf.longitudinalAcceleration.longitudinalAccelerationConfidence =
        AccelerationConfidence_unavailable;
    bvchf.curvature.curvatureValue    = CurvatureValue_unavailable;
    bvchf.curvature.curvatureConfidence = CurvatureConfidence_unavailable;
    bvchf.curvatureCalculationMode    = CurvatureCalculationMode_unavailable;
    bvchf.yawRate.yawRateValue        = YawRateValue_unavailable;
    bvchf.yawRate.yawRateConfidence   = YawRateConfidence_unavailable;

    return cam;
}

// ============================================================
//  DemoBaseApplLayer
// ============================================================

void DemoBaseApplLayer::initialize(int stage)
{
    BaseApplLayer::initialize(stage);

    if (stage == 0) {

        // initialize pointers to other modules
        if (FindModule<TraCIMobility*>::findSubModule(getParentModule())) {
            mobility = TraCIMobilityAccess().get(getParentModule());
            traci = mobility->getCommandInterface();
            traciVehicle = mobility->getVehicleCommandInterface();
        }
        else {
            traci = nullptr;
            mobility = nullptr;
            traciVehicle = nullptr;
        }

        annotations = AnnotationManagerAccess().getIfExists();
        ASSERT(annotations);

        mac = FindModule<DemoBaseApplLayerToMac1609_4Interface*>::findSubModule(getParentModule());
        ASSERT(mac);

        // read parameters
        headerLength      = par("headerLength");
        sendBeacons       = par("sendBeacons").boolValue();
        beaconLengthBits  = par("beaconLengthBits");
        beaconUserPriority = par("beaconUserPriority");
        beaconInterval    = par("beaconInterval");

        dataLengthBits    = par("dataLengthBits");
        dataOnSch         = par("dataOnSch").boolValue();
        dataUserPriority  = par("dataUserPriority");

        wsaInterval            = par("wsaInterval").doubleValue();
        currentOfferedServiceId = -1;

        isParked = false;

        findHost()->subscribe(BaseMobility::mobilityStateChangedSignal, this);
        findHost()->subscribe(TraCIMobility::parkingStateChangedSignal, this);

        sendBeaconEvt = new cMessage("beacon evt", SEND_BEACON_EVT);
        sendWSAEvt    = new cMessage("wsa evt",    SEND_WSA_EVT);
        sendCamEvt    = new cMessage("cam evt",    SEND_CAM_EVT);

        generatedBSMs = 0;
        generatedWSAs = 0;
        generatedWSMs = 0;
        generatedCAMs = 0;
        receivedBSMs  = 0;
        receivedWSAs  = 0;
        receivedWSMs  = 0;
        receivedCAMs  = 0;

        // --- Inizializza VehicleDataProvider Vanetza ---
        if (mobility) {
            mVehicleDataProvider = std::make_unique<VeinsVehicleDataProvider>(mobility);
        }
    }
    else if (stage == 1) {

        // store MAC address for quick access
        myId = mac->getMACAddress();

        if (dataOnSch == true && !mac->isChannelSwitchingActive()) {
            dataOnSch = false;
            EV_ERROR << "App wants to send data on SCH but MAC doesn't use any SCH. "
                        "Sending all data on CCH" << std::endl;
        }

        simtime_t firstBeacon = simTime();

        if (par("avoidBeaconSynchronization").boolValue() == true) {

            simtime_t randomOffset = dblrand() * beaconInterval;
            firstBeacon = simTime() + randomOffset;

            if (mac->isChannelSwitchingActive() == true) {
                if (beaconInterval.raw() % (mac->getSwitchingInterval().raw() * 2)) {
                    EV_ERROR << "The beacon interval (" << beaconInterval
                             << ") is smaller than or not a multiple of one synchronization interval ("
                             << 2 * mac->getSwitchingInterval()
                             << "). This means that beacons are generated during SCH intervals"
                             << std::endl;
                }
                firstBeacon = computeAsynchronousSendingTime(beaconInterval, ChannelType::control);
            }

            if (sendBeacons) {
                scheduleAt(firstBeacon, sendBeaconEvt);
            }
        }

        // CAM ETSI: primo invio dopo 1 s (poi ogni 100 ms per rispettare ETSI EN 302 637-2)
        scheduleAt(simTime() + 1.0, sendCamEvt);
    }
}

// ------------------------------------------------------------

simtime_t DemoBaseApplLayer::computeAsynchronousSendingTime(simtime_t interval, ChannelType chan)
{
    simtime_t randomOffset    = dblrand() * interval;
    simtime_t firstEvent;
    simtime_t switchingInterval = mac->getSwitchingInterval(); // usually 0.050 s
    simtime_t nextCCH;

    if (mac->isCurrentChannelCCH()) {
        nextCCH = simTime() - SimTime().setRaw(simTime().raw() % switchingInterval.raw())
                  + switchingInterval * 2;
    }
    else {
        nextCCH = simTime() - SimTime().setRaw(simTime().raw() % switchingInterval.raw())
                  + switchingInterval;
    }

    firstEvent = nextCCH + randomOffset;

    if (firstEvent.raw() % (2 * switchingInterval.raw()) > switchingInterval.raw()) {
        if (chan == ChannelType::control) firstEvent -= switchingInterval;
    }
    else {
        if (chan == ChannelType::service) firstEvent += switchingInterval;
    }

    return firstEvent;
}

// ------------------------------------------------------------

void DemoBaseApplLayer::populateWSM(BaseFrame1609_4* wsm, LAddress::L2Type rcvId, int serial)
{
    wsm->setRecipientAddress(rcvId);
    wsm->setBitLength(headerLength);

    if (DemoSafetyMessage* bsm = dynamic_cast<DemoSafetyMessage*>(wsm)) {
        bsm->setSenderPos(curPosition);
        bsm->setSenderSpeed(curSpeed);
        bsm->setPsid(-1);
        bsm->setChannelNumber(static_cast<int>(Channel::cch));
        bsm->addBitLength(beaconLengthBits);
        wsm->setUserPriority(beaconUserPriority);
    }
    else if (DemoServiceAdvertisment* wsa = dynamic_cast<DemoServiceAdvertisment*>(wsm)) {
        wsa->setChannelNumber(static_cast<int>(Channel::cch));
        wsa->setTargetChannel(static_cast<int>(currentServiceChannel));
        wsa->setPsid(currentOfferedServiceId);
        wsa->setServiceDescription(currentServiceDescription.c_str());
    }
    else if (CamMessage* cam = dynamic_cast<CamMessage*>(wsm)) {
        // Popola il wrapper OMNeT++ con il payload Vanetza
        cam->setChannelNumber(static_cast<int>(Channel::cch));
        cam->setPsid(-1);
        cam->addBitLength(beaconLengthBits);
        wsm->setUserPriority(beaconUserPriority);

        // Costruisci il CAM ETSI tramite Vanetza e salvalo nel messaggio
        if (mVehicleDataProvider) {
            vanetza::asn1::Cam vanetzaCam = buildCam(*mVehicleDataProvider);
            cam->setVanetzaCam(std::move(vanetzaCam)); // vedi nota (*)
        }
    }
    else {
        if (dataOnSch)
            wsm->setChannelNumber(static_cast<int>(Channel::sch1));
        else
            wsm->setChannelNumber(static_cast<int>(Channel::cch));
        wsm->addBitLength(dataLengthBits);
        wsm->setUserPriority(dataUserPriority);
    }
}

// ------------------------------------------------------------

void DemoBaseApplLayer::receiveSignal(cComponent* source, simsignal_t signalID,
                                      cObject* obj, cObject* details)
{
    Enter_Method_Silent();
    if (signalID == BaseMobility::mobilityStateChangedSignal) {
        handlePositionUpdate(obj);
    }
    else if (signalID == TraCIMobility::parkingStateChangedSignal) {
        handleParkingUpdate(obj);
    }
}

void DemoBaseApplLayer::handlePositionUpdate(cObject* obj)
{
    ChannelMobilityPtrType const mobility = check_and_cast<ChannelMobilityPtrType>(obj);
    curPosition = mobility->getPositionAt(simTime());
    curSpeed    = mobility->getCurrentSpeed();
}

void DemoBaseApplLayer::handleParkingUpdate(cObject* obj)
{
    isParked = mobility->getParkingState();
}

// ------------------------------------------------------------

void DemoBaseApplLayer::handleLowerMsg(cMessage* msg)
{
    BaseFrame1609_4* wsm = dynamic_cast<BaseFrame1609_4*>(msg);
    ASSERT(wsm);

    if (DemoSafetyMessage* bsm = dynamic_cast<DemoSafetyMessage*>(wsm)) {
        receivedBSMs++;
        onBSM(bsm);
    }
    else if (DemoServiceAdvertisment* wsa = dynamic_cast<DemoServiceAdvertisment*>(wsm)) {
        receivedWSAs++;
        onWSA(wsa);
    }
    else if (CamMessage* cam = dynamic_cast<CamMessage*>(wsm)) {
        receivedCAMs++;
        onCAM(cam);
    }
    else {
        receivedWSMs++;
        onWSM(wsm);
    }

    delete msg;
}

// ------------------------------------------------------------

void DemoBaseApplLayer::handleSelfMsg(cMessage* msg)
{
    switch (msg->getKind()) {

    case SEND_BEACON_EVT: {
        DemoSafetyMessage* bsm = new DemoSafetyMessage();
        populateWSM(bsm);
        sendDown(bsm);
        scheduleAt(simTime() + beaconInterval, sendBeaconEvt);
        break;
    }

    case SEND_WSA_EVT: {
        DemoServiceAdvertisment* wsa = new DemoServiceAdvertisment();
        populateWSM(wsa);
        sendDown(wsa);
        scheduleAt(simTime() + wsaInterval, sendWSAEvt);
        break;
    }

    case SEND_CAM_EVT: {
        // Costruisci e invia il CAM ETSI tramite Vanetza
        CamMessage* cam = new CamMessage();
        populateWSM(cam);
        sendDown(cam);

        // ETSI EN 302 637-2: intervallo CAM tra 100 ms e 1 s
        // Qui usiamo 100 ms (T_GenCam_Dcc minimo)
        scheduleAt(simTime() + 0.1, sendCamEvt);
        break;
    }

    default:
        if (msg) EV_WARN << "APP: Error: Got Self Message of unknown kind! Name: "
                         << msg->getName() << endl;
        break;
    }
}

// ------------------------------------------------------------

void DemoBaseApplLayer::finish()
{
    recordScalar("generatedWSMs", generatedWSMs);
    recordScalar("receivedWSMs",  receivedWSMs);

    recordScalar("generatedBSMs", generatedBSMs);
    recordScalar("receivedBSMs",  receivedBSMs);

    recordScalar("generatedWSAs", generatedWSAs);
    recordScalar("receivedWSAs",  receivedWSAs);

    recordScalar("generatedCAMs", generatedCAMs);
    recordScalar("receivedCAMs",  receivedCAMs);
}

DemoBaseApplLayer::~DemoBaseApplLayer()
{
    cancelAndDelete(sendBeaconEvt);
    cancelAndDelete(sendWSAEvt);
    cancelAndDelete(sendCamEvt);
    findHost()->unsubscribe(BaseMobility::mobilityStateChangedSignal, this);
}

// ------------------------------------------------------------

void DemoBaseApplLayer::startService(Channel channel, int serviceId,
                                     std::string serviceDescription)
{
    if (sendWSAEvt->isScheduled()) {
        throw cRuntimeError("Starting service although another service was already started");
    }

    mac->changeServiceChannel(channel);
    currentOfferedServiceId  = serviceId;
    currentServiceChannel    = channel;
    currentServiceDescription = serviceDescription;

    simtime_t wsaTime = computeAsynchronousSendingTime(wsaInterval, ChannelType::control);
    scheduleAt(wsaTime, sendWSAEvt);
}

void DemoBaseApplLayer::stopService()
{
    cancelEvent(sendWSAEvt);
    currentOfferedServiceId = -1;
}

// ------------------------------------------------------------

void DemoBaseApplLayer::sendDown(cMessage* msg)
{
    checkAndTrackPacket(msg);
    BaseApplLayer::sendDown(msg);
}

void DemoBaseApplLayer::sendDelayedDown(cMessage* msg, simtime_t delay)
{
    checkAndTrackPacket(msg);
    BaseApplLayer::sendDelayedDown(msg, delay);
}

void DemoBaseApplLayer::checkAndTrackPacket(cMessage* msg)
{
    if (dynamic_cast<DemoSafetyMessage*>(msg)) {
        EV_TRACE << "sending down a BSM" << std::endl;
        generatedBSMs++;
    }
    else if (dynamic_cast<DemoServiceAdvertisment*>(msg)) {
        EV_TRACE << "sending down a WSA" << std::endl;
        generatedWSAs++;
    }
    else if (dynamic_cast<CamMessage*>(msg)) {
        EV_TRACE << "sending down a CAM (Vanetza ETSI)" << std::endl;
        generatedCAMs++;
    }
    else if (dynamic_cast<BaseFrame1609_4*>(msg)) {
        EV_TRACE << "sending down a WSM" << std::endl;
        generatedWSMs++;
    }
}
