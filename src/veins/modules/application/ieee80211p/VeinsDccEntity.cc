#include "VeinsDccEntity.h"
#include <omnetpp.h>
#include <cmath>
#include <cstdlib>

using namespace omnetpp;

namespace veins {

// ── VeinsTransmitRateControl ──────────────────────────────────────────────────

// Returns the minimum allowed delay before a CAM can be transmitted.
// Fixed to T_GenCamMin (100 ms) regardless of channel conditions,
// as DCC congestion-based delay adaptation is not yet integrated.
vanetza::Clock::duration VeinsTransmitRateControl::delay(const vanetza::dcc::Transmission&)
{
    return T_GenCamMin;
}

// Returns the current CAM generation interval (mTGenCam).
// This value is dynamically adjusted by applyEtsiGenerationRules():
// reduced when vehicle dynamics change, reset to T_GenCamMax otherwise.
vanetza::Clock::duration VeinsTransmitRateControl::interval(const vanetza::dcc::Transmission&)
{
    return mTGenCam;
}

// Called by Vanetza after each transmission to allow DCC feedback.
// Currently a no-op — congestion-based interval adaptation not yet integrated.
void VeinsTransmitRateControl::notify(const vanetza::dcc::Transmission&)
{
}

bool VeinsTransmitRateControl::applyEtsiGenerationRules(double deltaHeading_deg,
                                                   double deltaPos_m,
                                                   double deltaSpeed_ms,
                                                   vanetza::Clock::duration elapsed)
{
    // Condition 1 (ETSI §6.1.3): elapsed time exceeds T_GenCamMin AND
    // at least one dynamics threshold is exceeded → dynamics-triggered CAM
    if (elapsed >= T_GenCamMin) {
        bool dynamicsChanged =
            (deltaHeading_deg > HEADING_THRESHOLD_DEG) ||
            (deltaPos_m       > POSITION_THRESHOLD_M)  ||
            (deltaSpeed_ms    > SPEED_THRESHOLD_MS);

        if (dynamicsChanged) {
            mTGenCam = elapsed;
            mNGenCamCount++;
            if (mNGenCamCount >= N_GenCam) {
                mTGenCam      = T_GenCamMax;
                mNGenCamCount = 0;
            }
            return true;
        }
    }

    // Condition 2 (ETSI §6.1.3): elapsed time exceeds current mTGenCam
    // → periodic fallback CAM; reset interval to T_GenCamMax
    if (elapsed >= mTGenCam) {
        mNGenCamCount = 0;
        mTGenCam      = T_GenCamMax;
        return true;
    }

    return false;
}

// ── VeinsChannelProbeProcessor ────────────────────────────────────────────────

// Stub: CBR measurement received from MAC layer but not yet processed.
// Future integration point for DCC congestion-based rate adaptation.
void VeinsChannelProbeProcessor::indicate(vanetza::dcc::ChannelLoad)
{
}

// ── VeinsDccEntity ────────────────────────────────────────────────────────────

vanetza::dcc::TransmitRateControl& VeinsDccEntity::transmit_rate_control()
{
    return mTrc;
}

vanetza::dcc::ChannelProbeProcessor& VeinsDccEntity::channel_probe_processor()
{
    return mCpp;
}

// Bridge method between VanetzaAdapter and VeinsTransmitRateControl.
// Receives pre-computed delta values from the Adapter and delegates
// the ETSI generation decision to the TRC component.
bool VeinsDccEntity::forwardCamGenerationCheck(double deltaHeading_deg,
                                         double deltaPos_m,
                                         double deltaSpeed_ms,
                                         vanetza::Clock::duration elapsed)
{
    return mTrc.applyEtsiGenerationRules(deltaHeading_deg, deltaPos_m, deltaSpeed_ms, elapsed);
}

// ── ASN.1 population ─────────────────────────────────────────────────────────

// Encode ms as big-endian byte array into the ASN.1 BER buffer.
// The minimum number of bytes needed is computed first (ts.size),
// then bytes are written from most significant to least significant.
void VeinsDccEntity::setTimestamp(TimestampIts_t& ts, uint64_t ms)
{
    if (ts.buf) free(ts.buf);
    ts.size = 0;
    uint64_t temp = ms;
    do { ts.size++; temp >>= 8; } while (temp > 0);

    ts.buf = (uint8_t*)malloc(ts.size);
    for (size_t i = 0; i < ts.size; ++i)
        ts.buf[ts.size - 1 - i] = (ms >> (8 * i)) & 0xFF;
}

vanetza::asn1::Cam VeinsDccEntity::buildCam(const VeinsVehicleDataProvider& vdp)
{
    vanetza::asn1::Cam cam;

    // --- ITS PDU Header ---
    cam->header.messageID       = ItsPduHeader__messageID_cam;
    cam->header.protocolVersion = 2;
    cam->header.stationID       = vdp.station_id();

    uint32_t now_ms = static_cast<uint32_t>(std::lround(simTime().dbl() * 1000.0)) % 65536;
    cam->cam.generationDeltaTime = static_cast<GenerationDeltaTime_t>(now_ms);

    // --- Basic Container: station type and reference position (1e-7 degrees) ---
    auto& basic = cam->cam.camParameters.basicContainer;
    basic.stationType = StationType_passengerCar;

    // Populate referencePosition using Vanetza's copy() helper (cam_functions.hpp).
    // PositionFix wraps lat/lon as boost::units GeoAngles; altitude is left default
    // (NaN) since it is unavailable in simulation → copy() maps it to AltitudeValue_unavailable.
    vanetza::PositionFix fix;
    fix.latitude  = vdp.latitude();
    fix.longitude = vdp.longitude();

    vanetza::facilities::copy(fix, basic.referencePosition);

    // --- High Frequency Container: speed (cm/s), heading (tenths of degrees) ---
    // Mandatory fields not available in simulation are set to ETSI "unavailable" sentinel values
    auto& hfc = cam->cam.camParameters.highFrequencyContainer;
    hfc.present = HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;
    auto& bvchf = hfc.choice.basicVehicleContainerHighFrequency;

    double spd_cm_s = vdp.speed().value() * 100.0;
    if (!std::isfinite(spd_cm_s) || spd_cm_s < 0.0 || spd_cm_s >= static_cast<double>(SpeedValue_unavailable))
        bvchf.speed.speedValue = SpeedValue_unavailable;
    else
        bvchf.speed.speedValue = static_cast<SpeedValue_t>(std::lround(spd_cm_s));
    bvchf.speed.speedConfidence = SpeedConfidence_unavailable;

    double heading_tenths = std::fmod(std::fmod(vdp.heading().value() * (180.0 / M_PI), 360.0) + 360.0, 360.0) * 10.0;
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

vanetza::asn1::Denm VeinsDccEntity::buildDenm(const VeinsVehicleDataProvider& vdp,
                                               uint16_t sequenceNumber,
                                               int causeCode,
                                               int subCauseCode)
{
    vanetza::asn1::Denm denm;

    // --- ITS PDU Header ---
    denm->header.messageID       = ItsPduHeader__messageID_denm;
    denm->header.protocolVersion = 2;
    denm->header.stationID       = vdp.station_id();

    // --- Management Container: actionID, timestamps, event position, validity ---
    // sequenceNumber is managed externally by DemoBaseApplLayer (application state)
    auto& mgmt = denm->denm.management;
    mgmt.actionID.originatingStationID = vdp.station_id();
    mgmt.actionID.sequenceNumber       = sequenceNumber;

    uint64_t its_now = static_cast<uint64_t>(std::lround(simTime().dbl() * 1000.0));
    setTimestamp(mgmt.detectionTime, its_now);
    setTimestamp(mgmt.referenceTime, its_now);

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

    // --- Situation Container: dynamically allocated, holds cause/subcause codes ---
    // informationQuality = 5 (range 0–7, higher = more reliable, ETSI EN 302 637-3)
    denm->denm.situation = (SituationContainer_t*)calloc(1, sizeof(SituationContainer_t));
    if (denm->denm.situation) {
        denm->denm.situation->informationQuality     = 5;
        denm->denm.situation->eventType.causeCode    = causeCode;
        denm->denm.situation->eventType.subCauseCode = subCauseCode;
    }

    return denm;
}

} // namespace veins
