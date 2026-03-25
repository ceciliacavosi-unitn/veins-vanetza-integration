#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"

using namespace veins;

// ============================================================
//  VeinsVehicleDataProvider
// ============================================================

VeinsVehicleDataProvider::VeinsVehicleDataProvider(BaseMobility* mob)
    : mMobility(mob) {}

vanetza::units::GeoAngle VeinsVehicleDataProvider::latitude() const {
    if (!mMobility) return 0.0 * boost::units::degree::degree;
    double lat_m = mMobility->getPositionAt(simTime()).y;
    double deg = 45.0 + (lat_m / 111000.0);
    return deg * boost::units::degree::degree;
}

vanetza::units::GeoAngle VeinsVehicleDataProvider::longitude() const {
    if (!mMobility) return 0.0 * boost::units::degree::degree;
    double lon_m = mMobility->getPositionAt(simTime()).x;
    double deg = 9.0 + (lon_m / 111000.0);
    return deg * boost::units::degree::degree;
}

vanetza::units::Velocity VeinsVehicleDataProvider::speed() const {
    using namespace boost::units;
    using namespace boost::units::si;
    if (!mMobility) return 0.0 * meter_per_second;
    if (auto traciMob = dynamic_cast<veins::TraCIMobility*>(mMobility)) {
        return traciMob->getSpeed() * meter_per_second;
    }
    double spd = mMobility->getCurrentSpeed().length();
    return spd * meter_per_second;
}

vanetza::units::Angle VeinsVehicleDataProvider::heading() const {
    using namespace boost::units;
    using namespace boost::units::si;
    if (!mMobility) return 0.0 * radian;
    double rad = 0.0;
    if (auto traciMob = dynamic_cast<veins::TraCIMobility*>(mMobility)) {
        rad = traciMob->getHeading().getRad();
    } else {
        auto ori = mMobility->getCurrentOrientation();
        rad = std::atan2(ori.y, ori.x);
    }
    const double two_pi = 2.0 * M_PI;
    double normalized_rad = std::fmod(std::fmod(rad, two_pi) + two_pi, two_pi);
    return normalized_rad * radian;
}

uint32_t VeinsVehicleDataProvider::station_id() const {
    if (!mMobility) return 0u;
    return static_cast<uint32_t>(mMobility->getId());
}

vanetza::Clock::time_point VeinsVehicleDataProvider::timestamp() const {
    using namespace std::chrono;
    auto ms = duration_cast<milliseconds>(duration<double>(simTime().dbl()));
    return vanetza::Clock::time_point(ms);
}

// ============================================================
//  Helpers CAM
// ============================================================

static vanetza::asn1::Cam buildCam(const VeinsVehicleDataProvider& vdp)
{
    vanetza::asn1::Cam cam;

    cam->header.messageID       = ItsPduHeader__messageID_cam;
    cam->header.protocolVersion = 2;
    cam->header.stationID       = vdp.station_id();

    uint32_t now_ms = static_cast<uint32_t>(std::lround(simTime().dbl() * 1000.0)) % 65536;
    cam->cam.generationDeltaTime = static_cast<GenerationDeltaTime_t>(now_ms);

    auto& basic = cam->cam.camParameters.basicContainer;
    basic.stationType = StationType_passengerCar;

    double lat_deg = vdp.latitude().value();
    double lon_deg = vdp.longitude().value();

    basic.referencePosition.latitude  = static_cast<Latitude_t>(std::lround(lat_deg * 1e7));
    basic.referencePosition.longitude = static_cast<Longitude_t>(std::lround(lon_deg * 1e7));
    basic.referencePosition.positionConfidenceEllipse.semiMajorConfidence  = SemiAxisLength_unavailable;
    basic.referencePosition.positionConfidenceEllipse.semiMinorConfidence  = SemiAxisLength_unavailable;
    basic.referencePosition.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
    basic.referencePosition.altitude.altitudeValue      = AltitudeValue_unavailable;
    basic.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;

    auto& hfc = cam->cam.camParameters.highFrequencyContainer;
    hfc.present = HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;
    auto& bvchf = hfc.choice.basicVehicleContainerHighFrequency;

    double speed_m_s = vdp.speed().value();
    double spd_cm_s  = speed_m_s * 100.0;
    if (!std::isfinite(spd_cm_s) || spd_cm_s < 0.0 || spd_cm_s >= static_cast<double>(SpeedValue_unavailable))
        bvchf.speed.speedValue = SpeedValue_unavailable;
    else
        bvchf.speed.speedValue = static_cast<SpeedValue_t>(std::lround(spd_cm_s));
    bvchf.speed.speedConfidence = SpeedConfidence_unavailable;

    double heading_rad    = vdp.heading().value();
    double heading_deg    = heading_rad * (180.0 / M_PI);
    double heading_tenths = std::fmod(std::fmod(heading_deg, 360.0) + 360.0, 360.0) * 10.0;
    if (!std::isfinite(heading_tenths))
        bvchf.heading.headingValue = HeadingValue_unavailable;
    else {
        heading_tenths = std::max(0.0, std::min(3600.0, heading_tenths));
        bvchf.heading.headingValue = static_cast<HeadingValue_t>(std::lround(heading_tenths));
    }
    bvchf.heading.headingConfidence = HeadingConfidence_unavailable;

    bvchf.driveDirection = DriveDirection_forward;
    bvchf.vehicleLength.vehicleLengthValue = VehicleLengthValue_unavailable;
    bvchf.vehicleLength.vehicleLengthConfidenceIndication = VehicleLengthConfidenceIndication_unavailable;
    bvchf.vehicleWidth = VehicleWidth_unavailable;
    bvchf.longitudinalAcceleration.longitudinalAccelerationValue      = LongitudinalAccelerationValue_unavailable;
    bvchf.longitudinalAcceleration.longitudinalAccelerationConfidence = AccelerationConfidence_unavailable;
    bvchf.curvature.curvatureValue      = CurvatureValue_unavailable;
    bvchf.curvature.curvatureConfidence = CurvatureConfidence_unavailable;
    bvchf.curvatureCalculationMode = CurvatureCalculationMode_unavailable;
    bvchf.yawRate.yawRateValue      = YawRateValue_unavailable;
    bvchf.yawRate.yawRateConfidence = YawRateConfidence_unavailable;

    return cam;
}

// ============================================================
//  Helpers DENM
// ============================================================

static void setTimestamp(TimestampIts_t& ts, uint64_t ms)
{
    if (ts.buf) free(ts.buf);
    ts.size = 0;
    uint64_t temp = ms;
    do { ts.size++; temp >>= 8; } while (temp > 0);
    ts.buf = (uint8_t*)malloc(ts.size);
    for (size_t i = 0; i < ts.size; ++i)
        ts.buf[ts.size - 1 - i] = (ms >> (8 * i)) & 0xFF;
}

static vanetza::asn1::Denm buildDenm(const VeinsVehicleDataProvider& vdp,
                                      uint16_t sequenceNumber,
                                      int causeCode,
                                      int subCauseCode)
{
    // LOG 1: ingresso in buildDenm
    EV_INFO << "[buildDenm] ENTER causeCode=" << causeCode
            << " subCauseCode=" << subCauseCode
            << " seq=" << sequenceNumber
            << " stationID=" << vdp.station_id() << "\n";

    vanetza::asn1::Denm denm;

    denm->header.messageID       = ItsPduHeader__messageID_denm;
    denm->header.protocolVersion = 2;
    denm->header.stationID       = vdp.station_id();

    auto& mgmt = denm->denm.management;
    mgmt.actionID.originatingStationID = vdp.station_id();
    mgmt.actionID.sequenceNumber       = sequenceNumber;

    uint64_t its_now = static_cast<uint64_t>(std::lround(simTime().dbl() * 1000.0));
    setTimestamp(mgmt.detectionTime,  its_now);
    setTimestamp(mgmt.referenceTime,  its_now);

    double lat_deg = vdp.latitude().value();
    double lon_deg = vdp.longitude().value();
    mgmt.eventPosition.latitude  = static_cast<Latitude_t>(std::lround(lat_deg * 1e7));
    mgmt.eventPosition.longitude = static_cast<Longitude_t>(std::lround(lon_deg * 1e7));
    mgmt.eventPosition.positionConfidenceEllipse.semiMajorConfidence  = SemiAxisLength_unavailable;
    mgmt.eventPosition.positionConfidenceEllipse.semiMinorConfidence  = SemiAxisLength_unavailable;
    mgmt.eventPosition.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
    mgmt.eventPosition.altitude.altitudeValue      = AltitudeValue_unavailable;
    mgmt.eventPosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;

    if (!mgmt.validityDuration)
        mgmt.validityDuration = (ValidityDuration_t*)calloc(1, sizeof(ValidityDuration_t));
    *mgmt.validityDuration = 600;
    mgmt.stationType = StationType_passengerCar;

    denm->denm.situation = (SituationContainer_t*)calloc(1, sizeof(SituationContainer_t));
    if (denm->denm.situation) {
        denm->denm.situation->informationQuality     = 5;
        denm->denm.situation->eventType.causeCode    = causeCode;
        denm->denm.situation->eventType.subCauseCode = subCauseCode;

        // LOG 2: conferma scrittura ASN.1
        EV_INFO << "[buildDenm] ASN.1 written:"
                << " causeCode=" << denm->denm.situation->eventType.causeCode
                << " subCauseCode=" << denm->denm.situation->eventType.subCauseCode
                << " informationQuality=" << denm->denm.situation->informationQuality << "\n";
    } else {
        EV_ERROR << "[buildDenm] situation calloc FAILED — causeCode not set!\n";
    }

    return denm;
}

// ============================================================
//  DemoBaseApplLayer
// ============================================================

void DemoBaseApplLayer::initialize(int stage)
{
    BaseApplLayer::initialize(stage);

    if (stage == 0) {
        EV_INFO << "initialize: host=" << findHost()->getFullPath()
                << " parent=" << getParentModule()->getFullPath() << endl;

        auto found = FindModule<BaseMobility*>::findSubModule(findHost());
        EV_INFO << "FindModule::findSubModule returned "
                << (found ? found->getFullName() : std::string("nullptr")) << endl;

        if (found) {
            mobility = static_cast<BaseMobility*>(found);
            if (auto tr = dynamic_cast<veins::TraCIMobility*>(found)) {
                traci        = tr->getCommandInterface();
                traciVehicle = tr->getVehicleCommandInterface();
                EV_INFO << "initialize: TraCIMobility found, traci available\n";
            } else {
                traci        = nullptr;
                traciVehicle = nullptr;
                EV_INFO << "initialize: BaseMobility found (no TraCI)\n";
            }
        } else {
            traci        = nullptr;
            mobility     = nullptr;
            traciVehicle = nullptr;
            EV_INFO << "initialize: no mobility found\n";
        }

        mVehicleDataProvider = std::make_unique<VeinsVehicleDataProvider>(mobility);
        EV_INFO << "VeinsVehicleDataProvider created for node=" << getParentModule()->getIndex()
                << " (mobility " << (mobility ? "present" : "absent") << ")\n";

        annotations = AnnotationManagerAccess().getIfExists();
        ASSERT(annotations);

        mac = FindModule<DemoBaseApplLayerToMac1609_4Interface*>::findSubModule(getParentModule());
        ASSERT(mac);

        headerLength       = par("headerLength");
        sendBeacons        = par("sendBeacons").boolValue();
        beaconLengthBits   = par("beaconLengthBits");
        beaconUserPriority = par("beaconUserPriority");
        beaconInterval     = par("beaconInterval");

        dataLengthBits   = par("dataLengthBits");
        dataOnSch        = par("dataOnSch").boolValue();
        dataUserPriority = par("dataUserPriority");

        wsaInterval             = par("wsaInterval").doubleValue();
        currentOfferedServiceId = -1;

        denmLengthBits   = par("denmLengthBits");
        denmUserPriority = par("denmUserPriority");

        // LOG 3: parametri DENM letti dall'ini
        EV_INFO << "[initialize] DENM params from ini:"
                << " denmDefaultCause=" << par("denmDefaultCause").intValue()
                << " denmDefaultSubcause=" << par("denmDefaultSubcause").intValue() << "\n";

        isParked           = false;
        denmSequenceNumber = 0;

        findHost()->subscribe(BaseMobility::mobilityStateChangedSignal, this);
        if (mobility) {
            mobility->subscribe(TraCIMobility::collisionSignal, this);
            mobility->subscribe(TraCIMobility::parkingStateChangedSignal, this);
            EV_INFO << "Subscribed to TraCIMobility signals on mobility module\n";
        } else {
            EV_WARN << "Mobility is NULL — collision/parking signals not subscribed\n";
        }

        sendBeaconEvt = new cMessage("beacon evt", SEND_BEACON_EVT);
        sendWSAEvt    = new cMessage("wsa evt",    SEND_WSA_EVT);
        sendCamEvt    = new cMessage("cam evt",    SEND_CAM_EVT);
        sendDenmEvt   = new cMessage("denm evt",   SEND_DENM_EVT);

        generatedBSMs  = 0; generatedWSAs  = 0; generatedWSMs  = 0;
        generatedCAMs  = 0; generatedDENMs = 0;
        receivedBSMs   = 0; receivedWSAs   = 0; receivedWSMs   = 0;
        receivedCAMs   = 0; receivedDENMs  = 0;
    }
    else if (stage == 1) {
        myId = mac->getMACAddress();

        if (dataOnSch == true && !mac->isChannelSwitchingActive()) {
            dataOnSch = false;
            EV_ERROR << "App wants to send data on SCH but MAC doesn't use any SCH. Sending all data on CCH\n";
        }

        simtime_t firstBeacon = simTime();
        if (par("avoidBeaconSynchronization").boolValue() == true) {
            simtime_t randomOffset = dblrand() * beaconInterval;
            firstBeacon = simTime() + randomOffset;
            if (mac->isChannelSwitchingActive() == true) {
                if (beaconInterval.raw() % (mac->getSwitchingInterval().raw() * 2)) {
                    EV_ERROR << "Beacon interval not a multiple of switching interval\n";
                }
                firstBeacon = computeAsynchronousSendingTime(beaconInterval, ChannelType::control);
            }
            if (sendBeacons) scheduleAt(firstBeacon, sendBeaconEvt);
        }

        if (!mVehicleDataProvider && mobility)
            mVehicleDataProvider = std::make_unique<VeinsVehicleDataProvider>(mobility);

        if (mVehicleDataProvider)
            scheduleAt(simTime(), sendCamEvt);
        else
            EV_INFO << "No VeinsVehicleDataProvider available — CAM will not be scheduled\n";
    }
}

simtime_t DemoBaseApplLayer::computeAsynchronousSendingTime(simtime_t interval, ChannelType chan)
{
    simtime_t randomOffset      = dblrand() * interval;
    simtime_t switchingInterval = mac->getSwitchingInterval();
    simtime_t nextCCH;

    if (mac->isCurrentChannelCCH())
        nextCCH = simTime() - SimTime().setRaw(simTime().raw() % switchingInterval.raw()) + switchingInterval * 2;
    else
        nextCCH = simTime() - SimTime().setRaw(simTime().raw() % switchingInterval.raw()) + switchingInterval;

    simtime_t firstEvent = nextCCH + randomOffset;
    if (firstEvent.raw() % (2 * switchingInterval.raw()) > switchingInterval.raw()) {
        if (chan == ChannelType::control) firstEvent -= switchingInterval;
    } else {
        if (chan == ChannelType::service) firstEvent += switchingInterval;
    }
    return firstEvent;
}

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
        if (mVehicleDataProvider) {
            auto vanetzaCam = buildCam(*mVehicleDataProvider);
            vanetza::ByteBuffer buffer = vanetzaCam.encode();
            cam->setVanetzaPayloadArraySize(static_cast<int>(buffer.size()));
            for (size_t i = 0; i < buffer.size(); ++i)
                cam->setVanetzaPayload(static_cast<int>(i), buffer[i]);
            cam->setByteLength(static_cast<int>(buffer.size()));
            cam->setBitLength(headerLength + cam->getByteLength() * 8);
            cam->setChannelNumber(static_cast<int>(Channel::cch));
            cam->setPsid(36);
            cam->setUserPriority(beaconUserPriority);
        } else {
            EV_WARN << "populateWSM: no VeinsVehicleDataProvider — CAM payload empty\n";
            cam->setVanetzaPayloadArraySize(0);
            cam->setByteLength(0);
        }
    }
    else if (DenmMessage* denm = dynamic_cast<DenmMessage*>(wsm)) {
        if (mVehicleDataProvider) {
            try {
                denmSequenceNumber++;
                int causeCode    = par("denmDefaultCause");
                int subCauseCode = par("denmDefaultSubcause");

                // LOG 4: valori letti dall'ini in populateWSM
                EV_INFO << "[populateWSM] DENM par read:"
                        << " denmDefaultCause=" << causeCode
                        << " denmDefaultSubcause=" << subCauseCode
                        << " seq=" << denmSequenceNumber
                        << " node=" << getParentModule()->getIndex() << "\n";

                auto vanetzaDenm = buildDenm(*mVehicleDataProvider, denmSequenceNumber,
                                             causeCode, subCauseCode);
                vanetza::ByteBuffer buffer = vanetzaDenm.encode();
                denm->setVanetzaPayloadArraySize(static_cast<int>(buffer.size()));
                for (size_t i = 0; i < buffer.size(); ++i)
                    denm->setVanetzaPayload(static_cast<int>(i), buffer[i]);
                denm->setByteLength(static_cast<int>(buffer.size()));

                EV_INFO << "[populateWSM] DENM encoded bytes=" << buffer.size() << "\n";
            } catch (...) {
                EV_WARN << "populateWSM: buildDenm failed — DENM payload empty\n";
                denm->setVanetzaPayloadArraySize(0);
                denm->setByteLength(0);
            }
        } else {
            denm->setVanetzaPayloadArraySize(0);
            denm->setByteLength(0);
        }
        denm->setChannelNumber(static_cast<int>(Channel::cch));
        denm->setPsid(-1);
        if (denm->getByteLength() == 0) denm->addBitLength(denmLengthBits);
        denm->setUserPriority(denmUserPriority);
    }
    else {
        if (dataOnSch) wsm->setChannelNumber(static_cast<int>(Channel::sch1));
        else           wsm->setChannelNumber(static_cast<int>(Channel::cch));
        wsm->addBitLength(dataLengthBits);
        wsm->setUserPriority(dataUserPriority);
    }
}

void DemoBaseApplLayer::receiveSignal(cComponent* source, simsignal_t signalID,
                                      cObject* obj, cObject* details)
{
    EV_INFO << "Signal received: " << signalID
            << " from " << source->getFullPath()
            << " at time " << simTime() << endl;
    Enter_Method_Silent();

    if (signalID == BaseMobility::mobilityStateChangedSignal) {
        handlePositionUpdate(obj);
    }
    else if (signalID == TraCIMobility::parkingStateChangedSignal) {
        int cause    = par("denmDefaultCause");
        int subcause = par("denmDefaultSubcause");
        EV_INFO << "[receiveSignal] Parking state changed:"
                << " causeCode=" << cause << " subCauseCode=" << subcause << endl;
        triggerDenm(static_cast<CauseCodeType_t>(cause), cause, subcause, nullptr, true);
        handleParkingUpdate(obj);
    }
    else if (signalID == TraCIMobility::collisionSignal) {
        int cause    = par("denmDefaultCause");
        int subcause = par("denmDefaultSubcause");
        EV_WARN << "[receiveSignal] TraCI Collision detected!"
                << " causeCode=" << cause << " subCauseCode=" << subcause << endl;
        triggerDenm(static_cast<CauseCodeType_t>(cause), cause, subcause, nullptr, true);
    }
    else {
        EV_DEBUG << "receiveSignal: unhandled signal id=" << signalID
                 << " source=" << source->getFullPath() << endl;
    }
}

void DemoBaseApplLayer::handlePositionUpdate(cObject* obj)
{
    ChannelMobilityPtrType const mob = check_and_cast<ChannelMobilityPtrType>(obj);
    curPosition      = mob->getPositionAt(simTime());
    double oldSpeed  = curSpeed.length();
    curSpeed         = mob->getCurrentSpeed();
    double newSpeed  = curSpeed.length();

    // Crea provider se non ancora disponibile (veicoli dinamici)
    if (!mVehicleDataProvider && this->mobility) {
        mVehicleDataProvider = std::make_unique<VeinsVehicleDataProvider>(this->mobility);
        if (!sendCamEvt->isScheduled())
            scheduleAt(simTime(), sendCamEvt);
    }

    // Diagnostic log: always print speed and cause to debug DENM triggering
    EV_INFO << "[handlePositionUpdate] oldSpeed=" << oldSpeed
            << " newSpeed=" << newSpeed
            << " cause=" << par("denmDefaultCause").intValue()
            << " node=" << getParentModule()->getIndex()
            << " t=" << simTime() << endl;

    // Rilevamento evento: veicolo fermo (non richiede oldSpeed > 1.0)
    // Usa solo newSpeed < 0.1 per catturare anche il primo aggiornamento dopo lo stop
    if (newSpeed < 0.1) {
        int cause    = par("denmDefaultCause");
        int subcause = par("denmDefaultSubcause");
        if (cause >= 0) {
            // Evita di inviare DENM se l'evento è già attivo
            if (activeDenmEvents.find(cause) == activeDenmEvents.end()) {
                EV_INFO << "[handlePositionUpdate] Veicolo fermo rilevato:"
                        << " causeCode=" << cause << " subCauseCode=" << subcause
                        << " node=" << getParentModule()->getIndex()
                        << " t=" << simTime() << endl;
                triggerDenm(static_cast<CauseCodeType_t>(cause), cause, subcause, nullptr, true);
            }
        } else {
            EV_WARN << "[handlePositionUpdate] denmDefaultCause=-1, DENM non inviato"
                    << " node=" << getParentModule()->getIndex() << endl;
        }
    } else {
        // Veicolo in movimento: resetta gli eventi attivi per permettere nuovi DENM
        int cause = par("denmDefaultCause");
        if (cause >= 0) {
            activeDenmEvents.erase(cause);
        }
    }
}

void DemoBaseApplLayer::handleParkingUpdate(cObject* obj)
{
    if (auto traciMob = dynamic_cast<veins::TraCIMobility*>(mobility))
        isParked = traciMob->getParkingState();
    else
        isParked = false;
}

void DemoBaseApplLayer::handleLowerMsg(cMessage* msg)
{
    BaseFrame1609_4* wsm = dynamic_cast<BaseFrame1609_4*>(msg);
    ASSERT(wsm);

    if (DemoSafetyMessage* bsm = dynamic_cast<DemoSafetyMessage*>(wsm)) {
        receivedBSMs++; onBSM(bsm);
    } else if (DemoServiceAdvertisment* wsa = dynamic_cast<DemoServiceAdvertisment*>(wsm)) {
        receivedWSAs++; onWSA(wsa);
    } else if (CamMessage* cam = dynamic_cast<CamMessage*>(wsm)) {
        receivedCAMs++; onCAM(cam);
    } else if (DenmMessage* denm = dynamic_cast<DenmMessage*>(wsm)) {
        receivedDENMs++; onDENM(denm);
    } else {
        receivedWSMs++; onWSM(wsm);
    }
    delete msg;
}

void DemoBaseApplLayer::handleSelfMsg(cMessage* msg)
{
    switch (msg->getKind()) {
    case SEND_BEACON_EVT: {
        DemoSafetyMessage* bsm = new DemoSafetyMessage();
        populateWSM(bsm);
        EV_INFO << "[TX] BEACON node=" << getParentModule()->getIndex()
                << " t=" << simTime() << endl;
        sendDown(bsm);
        scheduleAt(simTime() + beaconInterval, sendBeaconEvt);
        break;
    }
    case SEND_WSA_EVT: {
        DemoServiceAdvertisment* wsa = new DemoServiceAdvertisment();
        populateWSM(wsa);
        EV_INFO << "[TX] WSA node=" << getParentModule()->getIndex()
                << " t=" << simTime() << endl;
        sendDown(wsa);
        scheduleAt(simTime() + wsaInterval, sendWSAEvt);
        break;
    }
    case SEND_CAM_EVT: {
        if (!mVehicleDataProvider) {
            EV_WARN << "[TX] CAM: provider non disponibile node="
                    << getParentModule()->getIndex() << " retry in 0.1s\n";
            scheduleAt(simTime() + 0.1, sendCamEvt);
            break;
        }
        CamMessage* cam = new CamMessage();
        populateWSM(cam);
        EV_INFO << "[TX] CAM node=" << getParentModule()->getIndex()
                << " StationID=" << mVehicleDataProvider->station_id()
                << " t=" << simTime() << endl;
        sendDown(cam);
        scheduleAt(simTime() + 0.1, sendCamEvt);
        break;
    }
    case SEND_DENM_EVT: {
        if (!mVehicleDataProvider) {
            EV_WARN << "[TX] DENM: provider non disponibile node="
                    << getParentModule()->getIndex() << " skip\n";
            break;
        }
        DenmMessage* denm = new DenmMessage();
        populateWSM(denm);
        EV_INFO << "[TX] DENM node=" << getParentModule()->getIndex()
                << " t=" << simTime() << endl;
        sendDown(denm);
        break;
    }
    default:
        EV_WARN << "[TX] Messaggio self sconosciuto kind=" << msg->getKind()
                << " node=" << getParentModule()->getIndex() << endl;
        break;
    }
}

void DemoBaseApplLayer::onCAM(CamMessage* camMsg)
{
    size_t n = camMsg->getVanetzaPayloadArraySize();
    vanetza::ByteBuffer buffer(n);
    for (size_t i = 0; i < n; ++i)
        buffer[i] = static_cast<uint8_t>(camMsg->getVanetzaPayload(static_cast<int>(i)));
    try {
        vanetza::asn1::Cam cam;
        cam.decode(buffer);
        uint32_t id  = cam->header.stationID;
        double lat   = static_cast<double>(cam->cam.camParameters.basicContainer.referencePosition.latitude)  / 1e7;
        double lon   = static_cast<double>(cam->cam.camParameters.basicContainer.referencePosition.longitude) / 1e7;
        EV_INFO << ">>> RECEIVED CAM StationID=" << id
                << " POS=(" << lat << "," << lon << ") <<<\n";
    } catch (const std::exception& e) {
        EV_ERROR << "CAM decode error: " << e.what() << endl;
    }
}

void DemoBaseApplLayer::onDENM(DenmMessage* denmMsg)
{
    size_t n = denmMsg->getVanetzaPayloadArraySize();
    EV_INFO << "[RX] DENM node=" << getParentModule()->getIndex()
            << " payload=" << n << " bytes\n";
    if (n == 0) { EV_WARN << "[RX] DENM payload vuoto, skip\n"; return; }

    vanetza::ByteBuffer buffer(n);
    for (size_t i = 0; i < n; ++i)
        buffer[i] = static_cast<uint8_t>(denmMsg->getVanetzaPayload(static_cast<int>(i)));
    try {
        vanetza::asn1::Denm denm;
        denm.decode(buffer);

        int rxCause    = denm->denm.situation ? denm->denm.situation->eventType.causeCode    : -1;
        int rxSubCause = denm->denm.situation ? denm->denm.situation->eventType.subCauseCode : -1;

        // LOG 5: ricezione DENM con causeCode e subCauseCode decodificati
        EV_INFO << "[onDENM] >>> RECEIVED DENM <<<"
                << " StationID=" << denm->header.stationID
                << " causeCode=" << rxCause
                << " subCauseCode=" << rxSubCause
                << " node=" << getParentModule()->getIndex()
                << " t=" << simTime() << "\n";
    } catch (const std::exception& e) {
        EV_ERROR << "[RX] DENM decode error: " << e.what() << endl;
    }
}

void DemoBaseApplLayer::triggerDenm(CauseCodeType_t eventCause, int cause, int subcause,
                                     const Coord* eventPos, bool sendImmediate)
{
    int actualCause    = (cause >= 0)    ? cause    : static_cast<int>(eventCause);
    int actualSubcause = (subcause >= 0) ? subcause : 0;

    // LOG 6: ingresso in triggerDenm
    EV_INFO << "[triggerDenm] causeCode=" << actualCause
            << " subCauseCode=" << actualSubcause
            << " node=" << getParentModule()->getIndex()
            << " t=" << simTime() << "\n";

    if (actualCause < 0 || actualCause > 255) {
        EV_ERROR << "[triggerDenm] Invalid causeCode=" << actualCause << endl;
        return;
    }
    if (simTime() - lastDenmTime < denmMinInterval) {
        EV_WARN << "[triggerDenm] Rate-limited causeCode=" << actualCause << endl;
        return;
    }
    // Note: activeDenmEvents check is now handled in handlePositionUpdate
    // to allow re-triggering after vehicle resumes movement

    Coord pos = eventPos ? *eventPos : curPosition;
    lastDenmTime = simTime();
    activeDenmEvents.insert(actualCause);
    sendDenmNow(eventCause, cause, subcause, pos);

    EV_INFO << "[triggerDenm] DENM_TRIGGERED causeCode=" << actualCause
            << " subCauseCode=" << actualSubcause
            << " pos=(" << pos.x << "," << pos.y << ")"
            << " node=" << getParentModule()->getIndex() << "\n";
}

void DemoBaseApplLayer::sendDenmNow(CauseCodeType_t eventCause, int cause, int subcause, const Coord& eventPos)
{
    if (!mVehicleDataProvider) {
        EV_WARN << "[sendDenmNow] No VehicleDataProvider, aborting\n";
        return;
    }

    int causeCode    = (cause >= 0)    ? cause    : static_cast<int>(eventCause);
    int subCauseCode = (subcause >= 0) ? subcause : 0;

    // LOG 7: ingresso in sendDenmNow
    EV_INFO << "[sendDenmNow] >>> causeCode=" << causeCode
            << " subCauseCode=" << subCauseCode
            << " seq=" << (denmSequenceNumber + 1)
            << " node=" << getParentModule()->getIndex()
            << " t=" << simTime() << "\n";

    denmSequenceNumber++;
    vanetza::asn1::Denm denmAsn = buildDenm(*mVehicleDataProvider, denmSequenceNumber,
                                             causeCode, subCauseCode);

    vanetza::ByteBuffer buffer;
    try {
        buffer = denmAsn.encode();
    } catch (const std::exception& e) {
        EV_ERROR << "[sendDenmNow] encode failed: " << e.what() << "\n";
        return;
    } catch (...) {
        EV_ERROR << "[sendDenmNow] encode failed (unknown)\n";
        return;
    }

    DenmMessage* denmMsg = new DenmMessage("DENM");
    if (!buffer.empty()) {
        denmMsg->setVanetzaPayloadArraySize(static_cast<int>(buffer.size()));
        for (size_t i = 0; i < buffer.size(); ++i)
            denmMsg->setVanetzaPayload(static_cast<int>(i), buffer[i]);
        denmMsg->setByteLength(static_cast<int>(buffer.size()));
    } else {
        denmMsg->setVanetzaPayloadArraySize(0);
        denmMsg->setByteLength(0);
    }

    denmMsg->setStationId(myId);
    denmMsg->setCauseCode(static_cast<int16_t>(causeCode));
    denmMsg->setSubCause(static_cast<int16_t>(subCauseCode));
    denmMsg->setValidityDurationMs(0);
    denmMsg->setChannelNumber(static_cast<int>(Channel::cch));
    denmMsg->setPsid(-1);
    denmMsg->setUserPriority(denmUserPriority);
    denmMsg->setBitLength(headerLength + denmMsg->getByteLength() * 8);

    sendDown(denmMsg);
    lastDenmTime = simTime();
    activeDenmEvents.insert(causeCode);

    // LOG 8: conferma invio
    EV_INFO << "[sendDenmNow] DENM SENT causeCode=" << causeCode
            << " subCauseCode=" << subCauseCode
            << " bytes=" << buffer.size()
            << " node=" << getParentModule()->getIndex() << "\n";
}

void DemoBaseApplLayer::checkAndTrackPacket(cMessage* msg)
{
    if      (dynamic_cast<DemoSafetyMessage*>(msg))       { EV_TRACE << "sending down BSM\n";  generatedBSMs++;  }
    else if (dynamic_cast<DemoServiceAdvertisment*>(msg)) { EV_TRACE << "sending down WSA\n";  generatedWSAs++;  }
    else if (dynamic_cast<CamMessage*>(msg))              { EV_TRACE << "sending down CAM\n";  generatedCAMs++;  }
    else if (dynamic_cast<DenmMessage*>(msg))             { EV_TRACE << "sending down DENM\n"; generatedDENMs++; }
    else if (dynamic_cast<BaseFrame1609_4*>(msg))         { EV_TRACE << "sending down WSM\n";  generatedWSMs++;  }
}

void DemoBaseApplLayer::finish()
{
    recordScalar("generatedWSMs",  generatedWSMs);
    recordScalar("receivedWSMs",   receivedWSMs);
    recordScalar("generatedBSMs",  generatedBSMs);
    recordScalar("receivedBSMs",   receivedBSMs);
    recordScalar("generatedWSAs",  generatedWSAs);
    recordScalar("receivedWSAs",   receivedWSAs);
    recordScalar("generatedCAMs",  generatedCAMs);
    recordScalar("receivedCAMs",   receivedCAMs);
    recordScalar("generatedDENMs", generatedDENMs);
    recordScalar("receivedDENMs",  receivedDENMs);
}

DemoBaseApplLayer::~DemoBaseApplLayer()
{
    cancelAndDelete(sendBeaconEvt);
    cancelAndDelete(sendWSAEvt);
    cancelAndDelete(sendCamEvt);
    cancelAndDelete(sendDenmEvt);

    findHost()->unsubscribe(BaseMobility::mobilityStateChangedSignal, this);
    if (mobility) {
        mobility->unsubscribe(TraCIMobility::collisionSignal, this);
        mobility->unsubscribe(TraCIMobility::parkingStateChangedSignal, this);
    }
}

void DemoBaseApplLayer::startService(Channel channel, int serviceId, std::string serviceDescription)
{
    if (sendWSAEvt->isScheduled())
        throw cRuntimeError("Starting service although another service was already started");
    mac->changeServiceChannel(channel);
    currentOfferedServiceId   = serviceId;
    currentServiceChannel     = channel;
    currentServiceDescription = serviceDescription;
    scheduleAt(computeAsynchronousSendingTime(wsaInterval, ChannelType::control), sendWSAEvt);
}

void DemoBaseApplLayer::stopService()
{
    cancelEvent(sendWSAEvt);
    currentOfferedServiceId = -1;
}

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
