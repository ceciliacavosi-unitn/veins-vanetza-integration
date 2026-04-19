#pragma once

// Standard
#include <cstdint>

// Veins / OMNeT++
#include "veins/base/modules/BaseMobility.h" // contiene BaseMobility

// Vanetza types
#include "vanetza/common/clock.hpp"
#include "vanetza/units/angle.hpp"
#include "vanetza/units/velocity.hpp"
#include "vanetza/units/acceleration.hpp"

#include "veins/modules/mobility/traci/TraCIMobility.h"


namespace veins
{

using veins::TraCIMobility;

// ============================================================
//  VeinsVehicleDataProvider
//
//  Bridge between the Veins mobility module (BaseMobility /
//  TraCIMobility) and the Vanetza-style vehicle data interface.
//  Used by buildCam() and buildDenm() to read position, speed,
//  heading and station ID when populating message structures.
// ============================================================
class VEINS_API VeinsVehicleDataProvider {
public:
    explicit VeinsVehicleDataProvider(BaseMobility* mob);

    // Geographic position (converted from SUMO Cartesian metres)
    vanetza::units::GeoAngle latitude()  const;
    vanetza::units::GeoAngle longitude() const;

    // Kinematic data read from TraCIMobility (or fallback BaseMobility)
    vanetza::units::Velocity     speed()       const;
    vanetza::units::Angle        heading()     const;
    vanetza::units::Acceleration acceleration() const;

    // ITS station identifier (mapped from OMNeT++ module ID)
    uint32_t station_id() const;

    // Simulation timestamp converted to vanetza::Clock::time_point
    vanetza::Clock::time_point timestamp() const;

    // Optional: event cause/subcause for DENM generation
    int get_event_cause()    const;
    int get_event_subcause() const;

private:
    BaseMobility* mMobility; ///< Pointer to the underlying mobility module
};

} // namepspace veins
