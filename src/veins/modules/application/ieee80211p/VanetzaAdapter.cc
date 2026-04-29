#include "VanetzaAdapter.h"
#include <omnetpp.h>

using namespace omnetpp;

namespace veins {

VanetzaAdapter::VanetzaAdapter()
    : mDccEntity(std::make_unique<VeinsDccEntity>())
{
}

// ============================================================
// DCC entity access
// ============================================================

vanetza::dcc::Entity& VanetzaAdapter::getDccEntity()
{
    return *mDccEntity;
}

// ============================================================
// CAM generation check — ETSI EN 302 637-2 §6.1.3
// ============================================================

bool VanetzaAdapter::computeAndCheckCamDeltas(const VeinsVehicleDataProvider& vdp)
{
    if (!mLastCamValid) return true;

    ReferencePosition_t posNow{};
    posNow.latitude  = static_cast<Latitude_t>(std::lround(vdp.latitude().value()  * 1e7));
    posNow.longitude = static_cast<Longitude_t>(std::lround(vdp.longitude().value() * 1e7));
    posNow.altitude.altitudeValue      = AltitudeValue_unavailable;
    posNow.altitude.altitudeConfidence = AltitudeConfidence_unavailable;

    vanetza::units::Length deltaPos =
        vanetza::facilities::distance(posNow, mLastCamPos);
    double deltaPos_m = deltaPos / vanetza::units::si::meter;

    double headingNow_deg  = std::fmod(vdp.heading().value() * (180.0 / M_PI) + 360.0, 360.0);
    double headingLast_deg = std::fmod(mLastCamHeading / 10.0 + 360.0, 360.0);
    double deltaHeading_deg = std::abs(headingNow_deg - headingLast_deg);
    if (deltaHeading_deg > 180.0) deltaHeading_deg = 360.0 - deltaHeading_deg;

    double deltaSpeed_ms = std::abs(vdp.speed().value() - mLastCamSpeed_ms);

    auto now = vanetza::Clock::time_point(
        std::chrono::milliseconds(
            static_cast<long long>(simTime().dbl() * 1000.0)));
    auto elapsed = now - mLastCamTime;

    return mDccEntity->forwardCamGenerationCheck(
        deltaHeading_deg,
        deltaPos_m,
        deltaSpeed_ms,
        elapsed);
}

void VanetzaAdapter::notifyCamSent(const VeinsVehicleDataProvider& vdp)
{
    mLastCamPos.latitude  = static_cast<Latitude_t>(std::lround(vdp.latitude().value()  * 1e7));
    mLastCamPos.longitude = static_cast<Longitude_t>(std::lround(vdp.longitude().value() * 1e7));
    mLastCamPos.altitude.altitudeValue      = AltitudeValue_unavailable;
    mLastCamPos.altitude.altitudeConfidence = AltitudeConfidence_unavailable;

    double h_tenths = std::fmod(
        std::fmod(vdp.heading().value() * (180.0 / M_PI), 360.0) + 360.0, 360.0) * 10.0;
    mLastCamHeading = static_cast<HeadingValue_t>(std::lround(h_tenths));

    mLastCamSpeed_ms = vdp.speed().value();
    mLastCamTime = vanetza::Clock::time_point(
        std::chrono::milliseconds(
            static_cast<long long>(simTime().dbl() * 1000.0)));
    mLastCamValid = true;
}

// ============================================================
// Helper: DENM cause → string
// ============================================================

std::string VanetzaAdapter::denmCauseToString(int cause)
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
// POPULATE CAM — chiama mDccEntity->buildCam()
// ============================================================

void VanetzaAdapter::populateCAM(CamMessage* cam,
                                 VeinsVehicleDataProvider* vdp,
                                 int headerLength,
                                 int priority)
{
    if (!vdp) return;

    auto vanetzaCam = mDccEntity->buildCam(*vdp);   // ← ora in VeinsDccEntity
    vanetza::ByteBuffer buffer = vanetzaCam.encode();

    cam->setVanetzaPayloadArraySize(buffer.size());
    for (size_t i = 0; i < buffer.size(); ++i)
        cam->setVanetzaPayload(i, buffer[i]);

    cam->setByteLength(buffer.size());
    cam->setBitLength(headerLength + buffer.size() * 8);
    cam->setUserPriority(priority);
}

// ============================================================
// POPULATE DENM — chiama mDccEntity->buildDenm()
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

    auto vanetzaDenm = mDccEntity->buildDenm(*vdp, seq, cause, subcause);  // ← ora in VeinsDccEntity
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
