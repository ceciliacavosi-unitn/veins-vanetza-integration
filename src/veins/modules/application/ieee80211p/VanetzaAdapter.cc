#include "VanetzaAdapter.h"
#include <omnetpp.h>

using namespace omnetpp;

namespace veins {

// ============================================================
// Helper: DENM cause → string
// ============================================================

std::string VanetzaAdapter::denmCauseToString(int cause)
{
    switch (cause) {
        case 1: return "trafficCondition";
        case 2: return "accident";
        case 3: return "roadworks";
        case 5: return "impassability";
        case 6: return "adverseWeatherCondition_Adhesion";
        case 7: return "aquaplanning";
        case 9: return "hazardousLocation_SurfaceCondition";
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
// Timestamp helper
// ============================================================

void VanetzaAdapter::setTimestamp(TimestampIts_t& ts, uint64_t ms)
{
    if (ts.buf) free(ts.buf);
    ts.size = 0;
    uint64_t temp = ms;
    do { ts.size++; temp >>= 8; } while (temp > 0);

    ts.buf = (uint8_t*)malloc(ts.size);
    for (size_t i = 0; i < ts.size; ++i)
        ts.buf[ts.size - 1 - i] = (ms >> (8 * i)) & 0xFF;
}

// ============================================================
// BUILD CAM
// ============================================================

vanetza::asn1::Cam VanetzaAdapter::buildCam(const VeinsVehicleDataProvider& vdp)
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
// BUILD DENM
// ============================================================

vanetza::asn1::Denm VanetzaAdapter::buildDenm(const VeinsVehicleDataProvider& vdp,
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
// POPULATE CAM
// ============================================================

void VanetzaAdapter::populateCAM(CamMessage* cam,
                                 VeinsVehicleDataProvider* vdp,
                                 int headerLength,
                                 int priority)
{
    if (!vdp) return;

    auto vanetzaCam = buildCam(*vdp);
    vanetza::ByteBuffer buffer = vanetzaCam.encode();

    cam->setVanetzaPayloadArraySize(buffer.size());
    for (size_t i = 0; i < buffer.size(); ++i)
        cam->setVanetzaPayload(i, buffer[i]);

    cam->setByteLength(buffer.size());
    cam->setBitLength(headerLength + buffer.size() * 8);
    cam->setUserPriority(priority);
}

// ============================================================
// POPULATE DENM
// ============================================================

void VanetzaAdapter::populateDENM(DenmMessage* denm,
                                  VeinsVehicleDataProvider* vdp,
                                  uint16_t& seq,
                                  int cause,
                                  int subcause,
                                  int headerLength,
                                  int priority,
                                  int fallbackBits)
{
    if (!vdp) return;

    seq++;

    auto vanetzaDenm = buildDenm(*vdp, seq, cause, subcause);
    vanetza::ByteBuffer buffer = vanetzaDenm.encode();

    denm->setVanetzaPayloadArraySize(buffer.size());
    for (size_t i = 0; i < buffer.size(); ++i)
        denm->setVanetzaPayload(i, buffer[i]);

    denm->setByteLength(buffer.size());
    if (buffer.size() == 0)
        denm->addBitLength(fallbackBits);

    denm->setBitLength(headerLength + denm->getByteLength() * 8);
    denm->setUserPriority(priority);
}

// ============================================================
// RX CAM
// ============================================================

void VanetzaAdapter::onCAM(CamMessage* camMsg, const std::string& nodeName)
{
    size_t n = camMsg->getVanetzaPayloadArraySize();
    vanetza::ByteBuffer buffer(n);

    for (size_t i = 0; i < n; ++i)
        buffer[i] = camMsg->getVanetzaPayload(i);

    try {
        vanetza::asn1::Cam cam;
        cam.decode(buffer);

        EV_INFO << "CAM [RX]: stationID=" << cam->header.stationID
                << " name=" << nodeName
                << " t=" << simTime() << "\n";

    } catch (...) {
        EV_ERROR << "CAM decode error\n";
    }
}

// ============================================================
// RX DENM
// ============================================================

void VanetzaAdapter::onDENM(DenmMessage* denmMsg, const std::string& nodeName)
{
    size_t n = denmMsg->getVanetzaPayloadArraySize();
    if (n == 0) return;

    vanetza::ByteBuffer buffer(n);

    for (size_t i = 0; i < n; ++i)
        buffer[i] = denmMsg->getVanetzaPayload(i);

    try {
        vanetza::asn1::Denm denm;
        denm.decode(buffer);

        int cause = denm->denm.situation
            ? denm->denm.situation->eventType.causeCode
            : -1;

        EV_INFO << "DENM [RX]: cause=" << denmCauseToString(cause)
                << " name=" << nodeName
                << " t=" << simTime() << "\n";

    } catch (...) {
        EV_ERROR << "DENM decode error\n";
    }
}

} // namespace veins
