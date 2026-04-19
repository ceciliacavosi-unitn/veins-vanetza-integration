#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"

using namespace veins;

// ============================================================
//  Helper: DENM Cause Code to human-readable string
//  Used in EV_INFO / EV_WARN log messages throughout the file.
// ============================================================

static std::string denmCauseToString(int cause)
{
    switch (cause) {
        case 1:  return "trafficCondition";
        case 2:  return "accident";
        case 3:  return "roadworks";
        case 5:  return "impassability";
        case 6:  return "adverseWeatherCondition_Adhesion";
        case 7:  return "aquaplanning";
        case 9:  return "hazardousLocation_SurfaceCondition";
        case 10: return "hazardousLocation_ObstacleOnTheRoad";
        case 11: return "hazardousLocation_AnimalOnTheRoad";
        case 12: return "humanPresenceOnTheRoad";
        case 14: return "wrongWayDriving";
        case 15: return "rescueAndRecoveryWorkInProgress";
        case 17: return "adverseWeatherCondition_ExtremeWeather";
        case 18: return "adverseWeatherCondition_Visibility";
        case 19: return "adverseWeatherCondition_Precipitation";
        case 26: return "slowVehicle";
        case 27: return "dangerousEndOfQueue";
        case 91: return "vehicleBreakdown";
        case 92: return "postCrash";
        case 93: return "humanProblem";
        case 94: return "stationaryVehicle";
        case 95: return "emergencyVehicleApproaching";
        case 96: return "hazardousLocation_DangerousCurve";
        case 97: return "collisionRisk";
        case 98: return "signalViolation";
        case 99: return "dangerousSituation";
        default: return "unknown";
    }
}

// ============================================================
//  CAM builder (static helper)
//
//  TX PATH — called from populateWSM() when the outgoing
//  message is a CamMessage.
//
//  Fills the ASN.1 CAM structure with:
//    - ITS PDU header (messageID, protocolVersion, stationID)
//    - generationDeltaTime (current sim time mod 65536 ms)
//    - basicContainer: stationType + referencePosition
//    - highFrequencyContainer: speed, heading, and mandatory
//      unavailable fields (length, width, acceleration, etc.)
//
//  The returned vanetza::asn1::Cam is then encoded into a
//  ByteBuffer by populateWSM() and copied into vanetzaPayload.
// ============================================================

static vanetza::asn1::Cam buildCam(const VeinsVehicleDataProvider& vdp)
{
    vanetza::asn1::Cam cam;

    // --- ITS PDU Header ---
    cam->header.messageID       = ItsPduHeader__messageID_cam;
    cam->header.protocolVersion = 2;
    cam->header.stationID       = vdp.station_id();

    // generationDeltaTime: milliseconds since last full minute, mod 65536
    uint32_t now_ms = static_cast<uint32_t>(std::lround(simTime().dbl() * 1000.0)) % 65536;
    cam->cam.generationDeltaTime = static_cast<GenerationDeltaTime_t>(now_ms);

    // --- Basic Container ---
    auto& basic = cam->cam.camParameters.basicContainer;
    basic.stationType = StationType_passengerCar;

    double lat_deg = vdp.latitude().value();
    double lon_deg = vdp.longitude().value();

    // Position encoded in 1/10 micro-degrees (×1e7)
    basic.referencePosition.latitude  = static_cast<Latitude_t>(std::lround(lat_deg * 1e7));
    basic.referencePosition.longitude = static_cast<Longitude_t>(std::lround(lon_deg * 1e7));
    basic.referencePosition.positionConfidenceEllipse.semiMajorConfidence  = SemiAxisLength_unavailable;
    basic.referencePosition.positionConfidenceEllipse.semiMinorConfidence  = SemiAxisLength_unavailable;
    basic.referencePosition.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
    basic.referencePosition.altitude.altitudeValue      = AltitudeValue_unavailable;
    basic.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;

    // --- High Frequency Container (basicVehicleContainerHighFrequency) ---
    auto& hfc = cam->cam.camParameters.highFrequencyContainer;
    hfc.present = HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;
    auto& bvchf = hfc.choice.basicVehicleContainerHighFrequency;

    // Speed: encoded in cm/s; clamped to [0, SpeedValue_unavailable)
    double spd_cm_s = vdp.speed().value() * 100.0;
    if (!std::isfinite(spd_cm_s) || spd_cm_s < 0.0 || spd_cm_s >= static_cast<double>(SpeedValue_unavailable))
        bvchf.speed.speedValue = SpeedValue_unavailable;
    else
        bvchf.speed.speedValue = static_cast<SpeedValue_t>(std::lround(spd_cm_s));
    bvchf.speed.speedConfidence = SpeedConfidence_unavailable;

    // Heading: encoded in 1/10 degrees [0, 3600]; normalised from radians
    double heading_tenths = std::fmod(std::fmod(vdp.heading().value() * (180.0 / M_PI), 360.0) + 360.0, 360.0) * 10.0;
    if (!std::isfinite(heading_tenths))
        bvchf.heading.headingValue = HeadingValue_unavailable;
    else {
        heading_tenths = std::max(0.0, std::min(3600.0, heading_tenths));
        bvchf.heading.headingValue = static_cast<HeadingValue_t>(std::lround(heading_tenths));
    }
    bvchf.heading.headingConfidence = HeadingConfidence_unavailable;

    // Mandatory fields set to "unavailable" (standard-compliant placeholders)
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
//  DENM builder helpers (static)
//
//  TX PATH — called from populateWSM() when the outgoing
//  message is a DenmMessage.
// ============================================================

// Encodes a uint64_t millisecond timestamp into the variable-length
// ASN.1 TimestampIts_t BIT STRING (big-endian byte array).
// Only buildDENM uses it because a CAM message is periodic so it doesn't need the timestamp
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

// Builds the full ASN.1 DENM structure:
//   - ITS PDU header
//   - Management container: actionID, detectionTime, referenceTime,
//     eventPosition, validityDuration (600 s), stationType
//   - Situation container: informationQuality, causeCode, subCauseCode
//
// The returned vanetza::asn1::Denm is encoded into a ByteBuffer by
// populateWSM() and copied into vanetzaPayload.
static vanetza::asn1::Denm buildDenm(const VeinsVehicleDataProvider& vdp,
                                      uint16_t sequenceNumber,
                                      int causeCode,
                                      int subCauseCode)
{
    vanetza::asn1::Denm denm;

    // --- ITS PDU Header ---
    denm->header.messageID       = ItsPduHeader__messageID_denm;
    denm->header.protocolVersion = 2;
    denm->header.stationID       = vdp.station_id();

    // --- Management Container ---
    auto& mgmt = denm->denm.management;
    mgmt.actionID.originatingStationID = vdp.station_id();
    mgmt.actionID.sequenceNumber       = sequenceNumber; // incremented by populateWSM()

    uint64_t its_now = static_cast<uint64_t>(std::lround(simTime().dbl() * 1000.0));
    setTimestamp(mgmt.detectionTime, its_now);  // time the event was detected
    setTimestamp(mgmt.referenceTime, its_now);  // time this message was generated

    // Event position encoded in 1/10 micro-degrees (×1e7)
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
    *mgmt.validityDuration = 600; // seconds
    mgmt.stationType = StationType_passengerCar;

    // --- Situation Container ---
    denm->denm.situation = (SituationContainer_t*)calloc(1, sizeof(SituationContainer_t));
    if (denm->denm.situation) {
        denm->denm.situation->informationQuality     = 5;
        denm->denm.situation->eventType.causeCode    = causeCode;
        denm->denm.situation->eventType.subCauseCode = subCauseCode;
    } else {
        EV_ERROR << "DENM [BUILD]: situation container allocation failed — causeCode not set\n";
    }

    return denm;
}

// ============================================================
//  DemoBaseApplLayer — implementation
// ============================================================

// ============================================================
//  OMNeT++ lifecycle
// ============================================================

// Stage 0: resolves mobility, TraCI, MAC and annotation modules;
//          reads all .ini parameters; creates self-message timers;
//          subscribes to mobility/parking/collision signals.
// Stage 1: assigns MAC address; schedules first CAM event for
//          vehicles (TraCIMobility nodes) and first DENM event
//          for RSU nodes (non-TraCIMobility) using denmStartTime.
void DemoBaseApplLayer::initialize(int stage)
{
    BaseApplLayer::initialize(stage);

    if (stage == 0) {
        // --- Resolve mobility module ---
        auto found = FindModule<BaseMobility*>::findSubModule(findHost());
        if (found) {
            mobility = static_cast<BaseMobility*>(found);
            if (auto tr = dynamic_cast<veins::TraCIMobility*>(found)) {
                traci        = tr->getCommandInterface();
                traciVehicle = tr->getVehicleCommandInterface();
            } else {
                traci        = nullptr;
                traciVehicle = nullptr;
            }
        } else {
            traci        = nullptr;
            mobility     = nullptr;
            traciVehicle = nullptr;
        }

        // Create the Vanetza-compatible data provider (used by buildCam / buildDenm)
        mVehicleDataProvider = std::make_unique<VeinsVehicleDataProvider>(mobility);

        annotations = AnnotationManagerAccess().getIfExists();
        ASSERT(annotations);

        mac = FindModule<DemoBaseApplLayerToMac1609_4Interface*>::findSubModule(getParentModule());
        ASSERT(mac);

        // --- Read .ini parameters ---
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

        isParked           = false;
        denmSequenceNumber = 0;

        // --- Subscribe to signals ---
        findHost()->subscribe(BaseMobility::mobilityStateChangedSignal, this);
        if (mobility) {
            mobility->subscribe(TraCIMobility::collisionSignal, this);
            mobility->subscribe(TraCIMobility::parkingStateChangedSignal, this);
        }

        // --- Allocate self-message timers ---
        sendBeaconEvt = new cMessage("beacon evt", SEND_BEACON_EVT);
        sendWSAEvt    = new cMessage("wsa evt",    SEND_WSA_EVT);
        sendCamEvt    = new cMessage("cam evt",    SEND_CAM_EVT);
        sendDenmEvt   = new cMessage("denm evt",   SEND_DENM_EVT);

        // --- Reset statistics counters ---
        generatedBSMs  = 0; generatedWSAs  = 0; generatedWSMs  = 0;
        generatedCAMs  = 0; generatedDENMs = 0;
        receivedBSMs   = 0; receivedWSAs   = 0; receivedWSMs   = 0;
        receivedCAMs   = 0; receivedDENMs  = 0;
    }
    else if (stage == 1) {
        myId = mac->getMACAddress();

        // Disable SCH data if MAC is not doing channel switching
        if (dataOnSch == true && !mac->isChannelSwitchingActive()) {
            dataOnSch = false;
            EV_ERROR << "App wants to send data on SCH but MAC doesn't use any SCH. "
                        "Sending all data on CCH\n";
        }

        // Schedule first beacon with optional desynchronisation offset
        simtime_t firstBeacon = simTime();
        if (par("avoidBeaconSynchronization").boolValue() == true) {
            simtime_t randomOffset = dblrand() * beaconInterval;
            firstBeacon = simTime() + randomOffset;
            if (mac->isChannelSwitchingActive() == true) {
                if (beaconInterval.raw() % (mac->getSwitchingInterval().raw() * 2))
                    EV_ERROR << "Beacon interval not a multiple of switching interval\n";
                firstBeacon = computeAsynchronousSendingTime(beaconInterval, ChannelType::control);
            }
            if (sendBeacons) scheduleAt(firstBeacon, sendBeaconEvt);
        }

        // Lazily re-create the data provider if mobility was not yet available at stage 0
        if (!mVehicleDataProvider && mobility)
            mVehicleDataProvider = std::make_unique<VeinsVehicleDataProvider>(mobility);

        // CAM TX: schedule first event only for vehicle nodes (TraCIMobility)
        if (mVehicleDataProvider && mobility && dynamic_cast<veins::TraCIMobility*>(mobility))
            scheduleAt(simTime() + 1, sendCamEvt);

        // DENM TX (RSU): schedule first event at denmStartTime for non-vehicle nodes
        int cause = par("denmDefaultCause");
        if (cause >= 0 && !dynamic_cast<veins::TraCIMobility*>(mobility)) {
            double startTime = hasPar("denmStartTime") ? par("denmStartTime").doubleValue() : 10.0;
            double stopTime  = hasPar("denmStopTime")  ? par("denmStopTime").doubleValue()  : 200.0;
            EV_INFO << "RSU DENM [INIT]: cause=" << denmCauseToString(cause)
                    << " start=" << startTime << "s"
                    << " stop="  << stopTime  << "s"
                    << " duration=" << (stopTime - startTime) << "s"
                    << " name=" << getParentModule()->getFullName() << "\n";
            scheduleAt(simTime() + startTime, sendDenmEvt);
        }
    }
}

// Records all TX/RX counters as OMNeT++ scalars for post-simulation analysis.
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

// Cancels and deletes all self-message timers; unsubscribes from signals.
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

// ============================================================
//  TX PATH
// ============================================================

// ------------------------------------------------------------
//  TX — Self-message dispatcher automatically called by OMNeT++
//  framework when a self-message arrives to the module that is
//  treated as an alarm for the module to send anoter message
//
//  Central entry point for all periodic and one-shot TX events.
//
//  SEND_BEACON_EVT : creates BSM, populates, sends, reschedules.
//  SEND_WSA_EVT    : creates WSA, populates, sends, reschedules.
//  SEND_CAM_EVT    : creates CamMessage, calls populateWSM()
//                    which invokes buildCam() → vanetza encode →
//                    ByteBuffer → vanetzaPayload, then sendDown().
//                    Always rescheduled every 1 s (vehicles only).
//  SEND_DENM_EVT   : creates DenmMessage, calls populateWSM()
//                    which invokes buildDenm() → vanetza encode →
//                    ByteBuffer → vanetzaPayload, then sendDown().
//                    RSU: rescheduled periodically until denmStopTime.
//                    Vehicle: one-shot, NOT rescheduled here.
// ------------------------------------------------------------
void DemoBaseApplLayer::handleSelfMsg(cMessage* msg)
{
    switch (msg->getKind()) {

    // --- BSM / Beacon ---
    case SEND_BEACON_EVT: {
        DemoSafetyMessage* bsm = new DemoSafetyMessage();
        populateWSM(bsm);
        sendDown(bsm);
        scheduleAt(simTime() + beaconInterval, sendBeaconEvt);
        break;
    }

    // --- WSA ---
    case SEND_WSA_EVT: {
        DemoServiceAdvertisment* wsa = new DemoServiceAdvertisment();
        populateWSM(wsa);
        sendDown(wsa);
        scheduleAt(simTime() + wsaInterval, sendWSAEvt);
        break;
    }

    // --- CAM TX (vehicles only) ---
    case SEND_CAM_EVT: {
        // Guard: data provider must be ready (lazily created in handlePositionUpdate)
        if (!mVehicleDataProvider || !mobility) {
            EV_WARN << "CAM [TX]: VeinsVehicleDataProvider not ready — retrying in 1s"
                    << " name=" << getParentModule()->getFullName() << "\n";
            scheduleAt(simTime() + 1, sendCamEvt);
            break;
        }
        CamMessage* cam = new CamMessage();
        populateWSM(cam); // → buildCam() → vanetza encode → vanetzaPayload
        EV_INFO << "CAM [TX]: stationID=" << mVehicleDataProvider->station_id()
                << " pos=(" << mVehicleDataProvider->latitude().value()
                << ", "     << mVehicleDataProvider->longitude().value() << ")"
                << " speed=" << mVehicleDataProvider->speed().value() << " m/s"
                << " name=" << getParentModule()->getFullName()
                << " t=" << simTime() << "\n";
        sendDown(cam);
        scheduleAt(simTime() + 1, sendCamEvt); // periodic 1 Hz
        break;
    }

    // --- DENM TX ---
    case SEND_DENM_EVT: {
        if (!mVehicleDataProvider) {
            EV_WARN << "DENM [TX]: VeinsVehicleDataProvider not ready — skipping"
                    << " name=" << getParentModule()->getFullName() << "\n";
            break;
        }

        // Stop condition: do not send (or reschedule) past denmStopTime
        double stopTime = hasPar("denmStopTime") ? par("denmStopTime").doubleValue() : 200.0;
        if (simTime().dbl() > stopTime) {
            EV_INFO << "DENM [TX]: stopTime=" << stopTime << "s reached — stopping"
                    << " name=" << getParentModule()->getFullName() << "\n";
            break;
        }

        DenmMessage* denm = new DenmMessage();
        populateWSM(denm); // → buildDenm() → vanetza encode → vanetzaPayload
        EV_INFO << "DENM [TX]: stationID=" << mVehicleDataProvider->station_id()
                << " pos=(" << mVehicleDataProvider->latitude().value()
                << ", "     << mVehicleDataProvider->longitude().value() << ")"
                << " cause=" << denmCauseToString(par("denmDefaultCause"))
                << " name=" << getParentModule()->getFullName()
                << " t=" << simTime() << "\n";
        sendDown(denm);

        // RSU: reschedule periodically until stopTime.
        // Vehicle: one-shot — sendDenmNow() already fired this event; do NOT reschedule.
        if (!dynamic_cast<veins::TraCIMobility*>(mobility)) {
            double interval = hasPar("denmInterval") ? par("denmInterval").doubleValue() : 1.0;
            double nextTime = simTime().dbl() + interval;
            if (nextTime <= stopTime)
                scheduleAt(nextTime, sendDenmEvt);
            else
                EV_INFO << "DENM [TX]: next send would exceed stopTime — stopping\n";
        }
        break;
    }

    default:
        EV_WARN << "handleSelfMsg: unknown message kind=" << msg->getKind()
                << " name=" << getParentModule()->getFullName() << endl;
        break;
    }
}

// ------------------------------------------------------------
//  TX — Message population
//
//  Fills all fields of the outgoing BaseFrame1609_4 subclass.
//
//  CamMessage branch:
//    buildCam(*mVehicleDataProvider) → vanetza::asn1::Cam
//    cam.encode() → ByteBuffer → copied into vanetzaPayload[]
//
//  DenmMessage branch:
//    buildDenm(*mVehicleDataProvider, seqNum, cause, subcause)
//    → vanetza::asn1::Denm
//    denm.encode() → ByteBuffer → copied into vanetzaPayload[]
//
//  BSM / WSA / generic WSM: standard Veins field population.
// ------------------------------------------------------------
void DemoBaseApplLayer::populateWSM(BaseFrame1609_4* wsm, LAddress::L2Type rcvId, int serial)
{
    wsm->setRecipientAddress(rcvId);
    wsm->setBitLength(headerLength);

    if (DemoSafetyMessage* bsm = dynamic_cast<DemoSafetyMessage*>(wsm)) {
        // BSM: embed current position and speed for neighbour awareness
        bsm->setSenderPos(curPosition);
        bsm->setSenderSpeed(curSpeed);
        bsm->setPsid(-1);
        bsm->setChannelNumber(static_cast<int>(Channel::cch));
        bsm->addBitLength(beaconLengthBits);
        wsm->setUserPriority(beaconUserPriority);
    }
    else if (DemoServiceAdvertisment* wsa = dynamic_cast<DemoServiceAdvertisment*>(wsm)) {
        // WSA: advertise the currently offered service on CCH
        wsa->setChannelNumber(static_cast<int>(Channel::cch));
        wsa->setTargetChannel(static_cast<int>(currentServiceChannel));
        wsa->setPsid(currentOfferedServiceId);
        wsa->setServiceDescription(currentServiceDescription.c_str());
    }
    else if (CamMessage* cam = dynamic_cast<CamMessage*>(wsm)) {
        // CAM TX path:
        //   1. buildCam() fills the ASN.1 structure from VeinsVehicleDataProvider
        //   2. vanetza encodes it into a ByteBuffer
        //   3. ByteBuffer is copied byte-by-byte into vanetzaPayload[]
        if (mVehicleDataProvider) {
            auto vanetzaCam = buildCam(*mVehicleDataProvider);
            vanetza::ByteBuffer buffer = vanetzaCam.encode();
            cam->setVanetzaPayloadArraySize(static_cast<int>(buffer.size()));
            for (size_t i = 0; i < buffer.size(); ++i)
                cam->setVanetzaPayload(static_cast<int>(i), buffer[i]);
            cam->setByteLength(static_cast<int>(buffer.size()));
            cam->setBitLength(headerLength + cam->getByteLength() * 8);
            cam->setChannelNumber(static_cast<int>(Channel::cch));
            cam->setPsid(36); // ITS CAM PSID
            cam->setUserPriority(beaconUserPriority);
        } else {
            EV_WARN << "CAM [BUILD]: no VeinsVehicleDataProvider — payload empty\n";
            cam->setVanetzaPayloadArraySize(0);
            cam->setByteLength(0);
        }
    }
    else if (DenmMessage* denm = dynamic_cast<DenmMessage*>(wsm)) {
        // DENM TX path:
        //   1. denmSequenceNumber is incremented for each new DENM
        //   2. buildDenm() fills the ASN.1 structure (cause, subcause, position, timestamps)
        //   3. vanetza encodes it into a ByteBuffer
        //   4. ByteBuffer is copied byte-by-byte into vanetzaPayload[]
        if (mVehicleDataProvider) {
            try {
                denmSequenceNumber++;
                int causeCode    = par("denmDefaultCause");
                int subCauseCode = par("denmDefaultSubcause");

                auto vanetzaDenm = buildDenm(*mVehicleDataProvider, denmSequenceNumber,
                                             causeCode, subCauseCode);
                vanetza::ByteBuffer buffer = vanetzaDenm.encode();
                denm->setVanetzaPayloadArraySize(static_cast<int>(buffer.size()));
                for (size_t i = 0; i < buffer.size(); ++i)
                    denm->setVanetzaPayload(static_cast<int>(i), buffer[i]);
                denm->setByteLength(static_cast<int>(buffer.size()));
            } catch (...) {
                EV_WARN << "DENM [BUILD]: vanetza encoding failed — payload empty\n";
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
        // Generic WSM: route to SCH or CCH based on .ini configuration
        if (dataOnSch) wsm->setChannelNumber(static_cast<int>(Channel::sch1));
        else           wsm->setChannelNumber(static_cast<int>(Channel::cch));
        wsm->addBitLength(dataLengthBits);
        wsm->setUserPriority(dataUserPriority);
    }
}

// ------------------------------------------------------------
//  TX — DENM event-driven flow (vehicle nodes)
//
//  Signal dispatcher → handlePositionUpdate() evaluates trigger
//  conditions → triggerDenm() validates and rate-limits →
//  sendDenmNow() cancels any pending event and fires immediately.
// ------------------------------------------------------------

// Dispatches OMNeT++ signals to the appropriate handler:
//   mobilityStateChangedSignal → handlePositionUpdate()
//   parkingStateChangedSignal  → triggerDenm() + handleParkingUpdate()
//   collisionSignal            → triggerDenm()
//   automatically called by the framework OMNeT++
void DemoBaseApplLayer::receiveSignal(cComponent* source, simsignal_t signalID,
                                      cObject* obj, cObject* details)
{
    Enter_Method_Silent();

    if (signalID == BaseMobility::mobilityStateChangedSignal) {
        // check if there is any event in order to decide if we have to send a DENM or not
        handlePositionUpdate(obj);
    }
    else if (signalID == TraCIMobility::parkingStateChangedSignal) {
        // we are sure that a DENM need to be sent and we update also the internal state of the module
        int cause    = par("denmDefaultCause");
        int subcause = par("denmDefaultSubcause");
        triggerDenm(static_cast<CauseCodeType_t>(cause), cause, subcause, nullptr, true);
        handleParkingUpdate(obj);
    }
    else if (signalID == TraCIMobility::collisionSignal) {
        // we are sure that a DENM need to be sent
        int cause    = par("denmDefaultCause");
        int subcause = par("denmDefaultSubcause");
        triggerDenm(static_cast<CauseCodeType_t>(cause), cause, subcause, nullptr, true);
    }
}

// Called on every mobilityStateChangedSignal.
// Updates curPosition / curSpeed; lazily initialises mVehicleDataProvider.
// For vehicle nodes: evaluates per-cause trigger conditions and calls
// triggerDenm() when the condition is met and the rate-limit window has elapsed.
// Respects the global [denmStartTime, denmStopTime] window from .ini.
void DemoBaseApplLayer::handlePositionUpdate(cObject* obj)
{
    ChannelMobilityPtrType const mob = check_and_cast<ChannelMobilityPtrType>(obj);
    curPosition = mob->getPositionAt(simTime());
    curSpeed    = mob->getCurrentSpeed();

    // Lazily create data provider and schedule first CAM if not yet done
    if (!mVehicleDataProvider && this->mobility) {
        mVehicleDataProvider = std::make_unique<VeinsVehicleDataProvider>(this->mobility);
        if (!sendCamEvt->isScheduled()) scheduleAt(simTime(), sendCamEvt);
    }

    int cause = par("denmDefaultCause");
    if (cause < 0) return; // no DENM cause configured — nothing to do

    // Respect the global DENM time window
    double denmStart = hasPar("denmStartTime") ? par("denmStartTime").doubleValue() : 0.0;
    double denmStop  = hasPar("denmStopTime")  ? par("denmStopTime").doubleValue()  : 1e9;
    if (simTime().dbl() < denmStart || simTime().dbl() > denmStop) return;

    double speed_ms  = curSpeed.length();
    double speed_kmh = speed_ms * 3.6;
    bool   triggerCondition = false;
    double defaultInterval  = 1.0;

    // Helper: reads accidentStart and accidentDuration from the mobility module .ini params
    auto getAccidentWindow = [&](simtime_t& start, simtime_t& end) -> bool {
        cModule* mobMod = dynamic_cast<cModule*>(this->mobility);
        if (!mobMod || !mobMod->hasPar("accidentStart")) return false;
        start = mobMod->par("accidentStart").doubleValue();
        end   = start + simtime_t(mobMod->par("accidentDuration").doubleValue());
        return true;
    };

    // Helper: true if vehicle is stopped (< 0.1 m/s) within the accident time window
    auto stoppedInWindow = [&]() -> bool {
        simtime_t start, end;
        return getAccidentWindow(start, end) && speed_ms < 0.1
               && simTime() >= start && simTime() <= end;
    };

    // Helper: true if vehicle is below threshold_kmh within the accident time window
    auto slowInWindow = [&](double threshold_kmh) -> bool {
        simtime_t start, end;
        return getAccidentWindow(start, end) && speed_kmh < threshold_kmh
               && simTime() >= start && simTime() <= end;
    };

    // Evaluate trigger condition and default re-send interval per cause code
    switch (cause) {

        // --- Traffic / speed conditions ---
        case CauseCodeType_trafficCondition:
            if (speed_kmh < 30.0 && speed_kmh > 1.0) triggerCondition = true;
            defaultInterval = 2.0;
            break;
        case CauseCodeType_slowVehicle:
            if (slowInWindow(15.0) && speed_kmh > 0.5) triggerCondition = true;
            defaultInterval = 2.0;
            break;
        case CauseCodeType_dangerousEndOfQueue:
            if (speed_kmh < 10.0 && speed_kmh > 0.1) triggerCondition = true;
            defaultInterval = 1.0;
            break;

        // --- Stationary vehicle conditions (require stopped-in-window) ---
        case CauseCodeType_accident:
        case CauseCodeType_impassability:
        case CauseCodeType_vehicleBreakdown:
        case CauseCodeType_postCrash:
        case CauseCodeType_humanProblem:
        case CauseCodeType_stationaryVehicle:
            if (stoppedInWindow()) triggerCondition = true;
            defaultInterval = 1.0;
            break;
        case CauseCodeType_rescueAndRecoveryWorkInProgress:
            if (stoppedInWindow()) triggerCondition = true;
            defaultInterval = 2.0;
            break;

        // --- Environmental / weather conditions (always active within window) ---
        case CauseCodeType_adverseWeatherCondition_Adhesion:
        case CauseCodeType_adverseWeatherCondition_ExtremeWeatherCondition:
            triggerCondition = true;
            defaultInterval = 5.0;
            break;
        case CauseCodeType_aquaplannning:
        case CauseCodeType_adverseWeatherCondition_Visibility:
        case CauseCodeType_adverseWeatherCondition_Precipitation:
        case CauseCodeType_hazardousLocation_SurfaceCondition:
        case CauseCodeType_hazardousLocation_DangerousCurve:
            triggerCondition = true;
            defaultInterval = 3.0;
            break;

        // --- Localised hazards and dangerous behaviours ---
        case CauseCodeType_hazardousLocation_ObstacleOnTheRoad:
        case CauseCodeType_hazardousLocation_AnimalOnTheRoad:
        case CauseCodeType_humanPresenceOnTheRoad:
            triggerCondition = true;
            defaultInterval = 2.0;
            break;
        case CauseCodeType_wrongWayDriving:
        case CauseCodeType_emergencyVehicleApproaching:
        case CauseCodeType_collisionRisk:
        case CauseCodeType_signalViolation:
        case CauseCodeType_dangerousSituation:
            triggerCondition = true;
            defaultInterval = 1.0;
            break;

        default:
            break;
    }

    if (triggerCondition) {
        // Use explicit .ini denmInterval if provided, otherwise fall back to per-cause default
        double finalInterval = hasPar("denmInterval") ? par("denmInterval").doubleValue() : defaultInterval;
        if (simTime() - lastDenmTime >= finalInterval) {
            EV_INFO << "DENM [TRIGGER]: cause=" << denmCauseToString(cause)
                    << " speed=" << speed_kmh << " km/h"
                    << " interval=" << finalInterval << "s"
                    << " t=" << simTime() << endl;
            triggerDenm(static_cast<CauseCodeType_t>(cause), cause,
                        par("denmDefaultSubcause"), nullptr, true);
        }
    } else {
        // Condition no longer met: remove from active event set
        if (activeDenmEvents.count(cause)) {
            EV_INFO << "DENM [EVENT ENDED]: cause=" << denmCauseToString(cause) << endl;
            activeDenmEvents.erase(cause);
        }
    }
}

// Updates the isParked flag from the TraCIMobility parking state.
// Called from receiveSignal() on parkingStateChangedSignal.
void DemoBaseApplLayer::handleParkingUpdate(cObject* obj)
{
    if (auto traciMob = dynamic_cast<veins::TraCIMobility*>(mobility))
        isParked = traciMob->getParkingState();
    else
        isParked = false;
}

// Validates the event cause code, applies the denmMinInterval rate limit,
// registers the cause in activeDenmEvents, then delegates to sendDenmNow().
// Called from handlePositionUpdate() (speed/position trigger) or from
// receiveSignal() (parking / collision signals).
void DemoBaseApplLayer::triggerDenm(CauseCodeType_t eventCause, int cause, int subcause,
                                     const Coord* eventPos, bool sendImmediate)
{
    int actualCause    = (cause >= 0)    ? cause    : static_cast<int>(eventCause);
    int actualSubcause = (subcause >= 0) ? subcause : 0;

    // Validate cause code range (ITS-G5: 0–255)
    if (actualCause < 0 || actualCause > 255) {
        EV_ERROR << "DENM [TRIGGER]: invalid causeCode=" << actualCause << endl;
        return;
    }

    // Rate limiting: enforce minimum inter-DENM interval
    if (simTime() - lastDenmTime < denmMinInterval) {
        EV_WARN << "DENM [TRIGGER]: rate-limited — cause=" << denmCauseToString(actualCause)
                << " name=" << getParentModule()->getFullName() << endl;
        return;
    }

    Coord pos = eventPos ? *eventPos : curPosition;
    lastDenmTime = simTime();
    activeDenmEvents.insert(actualCause);
    sendDenmNow(eventCause, cause, subcause, pos);
}

// Cancels any pending sendDenmEvt and reschedules it at simTime() (immediate).
// handleSelfMsg(SEND_DENM_EVT) will then call populateWSM() → buildDenm() → sendDown().
// For vehicle nodes this is a one-shot fire; RSU periodic reschedule is
// handled inside handleSelfMsg(SEND_DENM_EVT).
void DemoBaseApplLayer::sendDenmNow(CauseCodeType_t eventCause, int cause, int subcause,
                                     const Coord& eventPos)
{
    if (!mVehicleDataProvider) {
        EV_WARN << "DENM [SEND NOW]: no VeinsVehicleDataProvider — aborted\n";
        return;
    }

    // Cancel any already-pending DENM event to avoid double-send
    if (sendDenmEvt->isScheduled())
        cancelEvent(sendDenmEvt);

    // Fire immediately; actual message construction happens in handleSelfMsg()
    scheduleAt(simTime(), sendDenmEvt);

    int causeCode = (cause >= 0) ? cause : static_cast<int>(eventCause);
    EV_INFO << "DENM [SEND NOW]: event scheduled → handleSelfMsg(SEND_DENM_EVT)"
            << " cause=" << denmCauseToString(causeCode)
            << " name=" << getParentModule()->getFullName() << "\n";
}

// ------------------------------------------------------------
//  TX — Send to MAC layer
// ------------------------------------------------------------

// Increments the appropriate generated* counter, then forwards
// the message down to BaseApplLayer::sendDown() → MAC layer.
void DemoBaseApplLayer::sendDown(cMessage* msg)
{
    checkAndTrackPacket(msg);
    BaseApplLayer::sendDown(msg);
}

// Delayed variant: increments counter then calls sendDelayedDown().
void DemoBaseApplLayer::sendDelayedDown(cMessage* msg, simtime_t delay)
{
    checkAndTrackPacket(msg);
    BaseApplLayer::sendDelayedDown(msg, delay);
}

// Inspects the dynamic type of the outgoing message and increments
// the matching generatedXxx counter (BSM, WSA, CAM, DENM, or generic WSM).
void DemoBaseApplLayer::checkAndTrackPacket(cMessage* msg)
{
    if      (dynamic_cast<DemoSafetyMessage*>(msg))       { generatedBSMs++;  }
    else if (dynamic_cast<DemoServiceAdvertisment*>(msg)) { generatedWSAs++;  }
    else if (dynamic_cast<CamMessage*>(msg))              { generatedCAMs++;  }
    else if (dynamic_cast<DenmMessage*>(msg))             { generatedDENMs++; }
    else if (dynamic_cast<BaseFrame1609_4*>(msg))         { generatedWSMs++;  }
}

// ------------------------------------------------------------
//  TX — WSA service management
// ------------------------------------------------------------

// Starts advertising a service on the given channel.
// Schedules the first WSA using a desynchronised sending time to
// avoid simultaneous WSA bursts from multiple nodes.
void DemoBaseApplLayer::startService(Channel channel, int serviceId,
                                      std::string serviceDescription)
{
    if (sendWSAEvt->isScheduled())
        throw cRuntimeError("Starting service although another service was already started");
    mac->changeServiceChannel(channel);
    currentOfferedServiceId   = serviceId;
    currentServiceChannel     = channel;
    currentServiceDescription = serviceDescription;
    scheduleAt(computeAsynchronousSendingTime(wsaInterval, ChannelType::control), sendWSAEvt);
}

// Stops the currently advertised service and cancels the WSA timer.
void DemoBaseApplLayer::stopService()
{
    cancelEvent(sendWSAEvt);
    currentOfferedServiceId = -1;
}

// ============================================================
//  RX PATH
// ============================================================

// ------------------------------------------------------------
//  RX — Lower message dispatcher
//
//  Entry point for all messages arriving from the MAC layer.
//  Casts the message to the correct subtype and dispatches to
//  the appropriate on*() callback; increments the matching
//  receivedXxx counter; deletes the message when done.
// ------------------------------------------------------------
void DemoBaseApplLayer::handleLowerMsg(cMessage* msg)
{
    BaseFrame1609_4* wsm = dynamic_cast<BaseFrame1609_4*>(msg);
    ASSERT(wsm);

    if (DemoSafetyMessage* bsm = dynamic_cast<DemoSafetyMessage*>(wsm)) {
        receivedBSMs++;  onBSM(bsm);
    } else if (DemoServiceAdvertisment* wsa = dynamic_cast<DemoServiceAdvertisment*>(wsm)) {
        receivedWSAs++;  onWSA(wsa);
    } else if (CamMessage* cam = dynamic_cast<CamMessage*>(wsm)) {
        receivedCAMs++;  onCAM(cam);
    } else if (DenmMessage* denm = dynamic_cast<DenmMessage*>(wsm)) {
        receivedDENMs++; onDENM(denm);
    } else {
        receivedWSMs++;  onWSM(wsm);
    }
    delete msg;
}

// ------------------------------------------------------------
//  RX — Message callbacks
// ------------------------------------------------------------

// CAM RX path:
//   1. Reads vanetzaPayload[] into a ByteBuffer
//   2. cam.decode(buffer) deserialises the ASN.1 structure
//   3. Extracts stationID, latitude, longitude, speed for logging
void DemoBaseApplLayer::onCAM(CamMessage* camMsg)
{
    size_t n = camMsg->getVanetzaPayloadArraySize();
    vanetza::ByteBuffer buffer(n);
    for (size_t i = 0; i < n; ++i)
        buffer[i] = static_cast<uint8_t>(camMsg->getVanetzaPayload(static_cast<int>(i)));
    try {
        vanetza::asn1::Cam cam;
        cam.decode(buffer); // deserialise ASN.1 CAM from ByteBuffer
        uint32_t id = cam->header.stationID;
        double lat  = static_cast<double>(cam->cam.camParameters.basicContainer
                          .referencePosition.latitude)  / 1e7;
        double lon  = static_cast<double>(cam->cam.camParameters.basicContainer
                          .referencePosition.longitude) / 1e7;
        double spd  = static_cast<double>(cam->cam.camParameters.highFrequencyContainer
                          .choice.basicVehicleContainerHighFrequency.speed.speedValue) / 100.0;
        EV_INFO << "CAM [RX]: stationID=" << id
                << " pos=(" << lat << ", " << lon << ")"
                << " speed=" << spd << " m/s"
                << " name=" << getParentModule()->getFullName()
                << " t=" << simTime() << "\n";
    } catch (const std::exception& e) {
        EV_ERROR << "CAM [RX]: decode error: " << e.what() << endl;
    }
}

// DENM RX path:
//   1. Guards against empty payload (RSU may send before vanetza is ready)
//   2. Reads vanetzaPayload[] into a ByteBuffer
//   3. denm.decode(buffer) deserialises the ASN.1 structure
//   4. Extracts stationID, causeCode, subCauseCode for logging
void DemoBaseApplLayer::onDENM(DenmMessage* denmMsg)
{
    size_t n = denmMsg->getVanetzaPayloadArraySize();
    if (n == 0) {
        EV_WARN << "DENM [RX]: empty payload — skipping"
                << " name=" << getParentModule()->getFullName() << "\n";
        return;
    }

    vanetza::ByteBuffer buffer(n);
    for (size_t i = 0; i < n; ++i)
        buffer[i] = static_cast<uint8_t>(denmMsg->getVanetzaPayload(static_cast<int>(i)));
    try {
        vanetza::asn1::Denm denm;
        denm.decode(buffer); // deserialise ASN.1 DENM from ByteBuffer
        int rxCause    = denm->denm.situation ? denm->denm.situation->eventType.causeCode    : -1;
        int rxSubCause = denm->denm.situation ? denm->denm.situation->eventType.subCauseCode : -1;
        EV_INFO << "DENM [RX]: stationID=" << denm->header.stationID
                << " cause=" << denmCauseToString(rxCause)
                << " (" << rxCause << "," << rxSubCause << ")"
                << " bytes=" << n
                << " name=" << getParentModule()->getFullName()
                << " t=" << simTime() << "\n";
    } catch (const std::exception& e) {
        EV_ERROR << "DENM [RX]: decode error: " << e.what() << endl;
    }
}

// ============================================================
//  Utility
// ============================================================

// Computes a desynchronised first-send time to avoid simultaneous
// beacon / WSA bursts from multiple nodes at simulation start.
// Aligns the offset to the correct CCH/SCH slot boundary when
// channel switching is active.
simtime_t DemoBaseApplLayer::computeAsynchronousSendingTime(simtime_t interval, ChannelType chan)
{
    simtime_t randomOffset      = dblrand() * interval;
    simtime_t switchingInterval = mac->getSwitchingInterval();
    simtime_t nextCCH;

    if (mac->isCurrentChannelCCH())
        nextCCH = simTime() - SimTime().setRaw(simTime().raw() % switchingInterval.raw())
                  + switchingInterval * 2;
    else
        nextCCH = simTime() - SimTime().setRaw(simTime().raw() % switchingInterval.raw())
                  + switchingInterval;

    simtime_t firstEvent = nextCCH + randomOffset;
    if (firstEvent.raw() % (2 * switchingInterval.raw()) > switchingInterval.raw()) {
        if (chan == ChannelType::control) firstEvent -= switchingInterval;
    } else {
        if (chan == ChannelType::service) firstEvent += switchingInterval;
    }
    return firstEvent;
}
