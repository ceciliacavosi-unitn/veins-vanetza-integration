// ============================================================
//  VeinsVehicleDataProvider — implementation
//
//  Bridges Veins BaseMobility / TraCIMobility to the
//  Vanetza-style data interface consumed by buildCam() and
//  buildDenm().  All geographic conversions assume a flat-
//  earth approximation centred on (45°N, 9°E) — sufficient
//  for small-scale SUMO scenarios.
// ============================================================

#include "veins/modules/application/ieee80211p/VeinsVehicleDataProvider.h"

using namespace omnetpp;

Define_Module(veins::VeinsVehicleDataProvider);

namespace veins {

// Default constructor required by Define_Module / OMNeT++ module system
VeinsVehicleDataProvider::VeinsVehicleDataProvider()
    : mMobility(nullptr)
{
}

VeinsVehicleDataProvider::VeinsVehicleDataProvider(BaseMobility* mob)
: mMobility(mob)
{
    const char *hostName = nullptr;
    if (mMobility && mMobility->getParentModule())
        hostName = mMobility->getParentModule()->getFullName();
    EV_INFO << "VeinsVehicleDataProvider constructed; mobility=" << (void*)mMobility
            << " host=" << (hostName ? hostName : "null") << " t=" << simTime() << "\n";
}

void VeinsVehicleDataProvider::initialize(int stage)
{
    cSimpleModule::initialize(stage);

    if (stage == 0) {
        mMobility = FindModule<BaseMobility*>::findSubModule(getParentModule());
        if (!mMobility) {
            EV_WARN << "VeinsVehicleDataProvider: mobility not found in host "
                    << getParentModule()->getFullName() << "\n";
        } else {
            EV_INFO << "VeinsVehicleDataProvider initialized as OMNeT++ module at "
                    << getFullPath() << "\n";
        }
    }
}

void VeinsVehicleDataProvider::handleMessage(omnetpp::cMessage* msg)
{
    // VeinsVehicleDataProvider does not receive messages directly.
    // All data is read on demand via getter methods.
    EV_WARN << "VeinsVehicleDataProvider: unexpected message received: "
            << msg->getName() << "\n";
    delete msg;
}

// Converts SUMO Y-coordinate (metres north of origin) to decimal degrees latitude.
vanetza::units::GeoAngle VeinsVehicleDataProvider::latitude() const {
    if (!mMobility) return 0.0 * boost::units::degree::degree;
    double lat_m = mMobility->getPositionAt(simTime()).y;
    double deg = 45.0 + (lat_m / 111000.0);
    return deg * boost::units::degree::degree;
}

// Converts SUMO X-coordinate (metres east of origin) to decimal degrees longitude.
vanetza::units::GeoAngle VeinsVehicleDataProvider::longitude() const {
    if (!mMobility) return 0.0 * boost::units::degree::degree;
    double lon_m = mMobility->getPositionAt(simTime()).x;
    double deg = 9.0 + (lon_m / 111000.0);
    return deg * boost::units::degree::degree;
}

// Returns speed in m/s.  Prefers TraCIMobility::getSpeed() for accuracy;
// falls back to BaseMobility::getCurrentSpeed().length().
vanetza::units::Velocity VeinsVehicleDataProvider::speed() const {
    using namespace boost::units;
    using namespace boost::units::si;
    if (!mMobility) return 0.0 * meter_per_second;
    if (auto traciMob = dynamic_cast<veins::TraCIMobility*>(mMobility))
        return traciMob->getSpeed() * meter_per_second;
    return mMobility->getCurrentSpeed().length() * meter_per_second;
}

// Returns heading in radians [0, 2π), normalised from TraCIMobility or
// computed via atan2 from the orientation vector for generic mobility.
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

// Maps the OMNeT++ module ID to an ITS StationID (uint32_t).
uint32_t VeinsVehicleDataProvider::station_id() const {
    if (!mMobility) return 0u;
    return static_cast<uint32_t>(mMobility->getId());
}

// Converts the current simulation time to a vanetza::Clock::time_point
// (millisecond resolution) for use in CAM generationDeltaTime and
// DENM detectionTime / referenceTime fields.
vanetza::Clock::time_point VeinsVehicleDataProvider::timestamp() const {
    using namespace std::chrono;
    auto ms = duration_cast<milliseconds>(duration<double>(simTime().dbl()));
    return vanetza::Clock::time_point(ms);
}

} // namespace veins
