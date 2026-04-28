#pragma once

#include <memory>
#include <string>

#include "veins/modules/application/ieee80211p/VeinsVehicleDataProvider.h"
#include "veins/modules/application/ieee80211p/VeinsDccEntity.h"

// Vanetza
#include "vanetza/asn1/cam.hpp"
#include "vanetza/asn1/denm.hpp"
#include "vanetza/asn1/asn1c_wrapper.hpp"
#include "vanetza/common/clock.hpp"
#include "vanetza/units/angle.hpp"
#include "vanetza/units/velocity.hpp"
#include "vanetza/units/acceleration.hpp"
#include "vanetza/asn1/its/CauseCodeType.h"
#include <vanetza/facilities/cam_functions.hpp>
#include <vanetza/asn1/its/ReferencePosition.h>
#include <vanetza/asn1/its/Heading.h>

// Veins messages
#include "veins/modules/messages/CamMessage_m.h"
#include "veins/modules/messages/DenmMessage_m.h"

namespace veins {

class CamMessage;
class DenmMessage;

class VanetzaAdapter : public cSimpleModule
{
public:
    VanetzaAdapter();
    ~VanetzaAdapter() = default;

    // ============================
    // TX helpers
    // ============================

    // build ASN.1 CAM
    vanetza::asn1::Cam buildCam(const VeinsVehicleDataProvider& vdp);

    // build ASN.1 DENM
    vanetza::asn1::Denm buildDenm(const VeinsVehicleDataProvider& vdp,
                                  uint16_t sequenceNumber,
                                  int causeCode,
                                  int subCauseCode);

    // populate only CAM/DENM parts (called from DemoBaseApplLayer)
    void populateCAM(CamMessage* cam,
                     VeinsVehicleDataProvider* vdp,
                     int headerLength,
                     int beaconUserPriority);

    void populateDENM(DenmMessage* denm,
                      VeinsVehicleDataProvider* vdp,
                      uint16_t& sequenceNumber,
                      int causeCode,
                      int subCauseCode,
                      int headerLength,
                      int denmUserPriority,
                      int denmLengthBits);

    // ============================
    // RX helpers
    // ============================

    void onCAM(CamMessage* camMsg, const std::string& nodeName);
    void onDENM(DenmMessage* denmMsg, const std::string& nodeName);

    // ============================
    // DCC access (for scheduling)
    // ============================

    /**
     * Returns the DCC TransmitRateControl interface.
     * Use this in DemoBaseApplLayer to get the CAM interval:
     *   auto interval = mAdapter->dccEntity().transmit_rate_control().interval();
     */
    vanetza::dcc::Entity& getDccEntity();

    // ============================
    // DCC / CAM generation check
    // ============================

    /**
     * Extracts vehicle state and computes deltas (position, heading, speed)
     * using Vanetza utilities, then forwards them to the DCC layer.
     *
     * Does not implement ETSI decision logic.
     */
    bool checkCamGeneration(const VeinsVehicleDataProvider& vdp);

    /**
     * Must be called after each CAM is actually sent.
     * Resets the last-CAM reference state (position, speed, heading, time).
     */
    void notifyCamSent(const VeinsVehicleDataProvider& vdp);

    // DENM message cause code to string
    std::string denmCauseToString(int cause);

private:
    void setTimestamp(TimestampIts_t& ts, uint64_t ms);
    std::unique_ptr<VeinsDccEntity> mDccEntity;

    // --- Last CAM reference state (for ETSI §6.1.3 delta checks) ---
    // Uses Vanetza ASN.1 types so similar_heading() and distance() can be called directly
    ReferencePosition_t         mLastCamPos;             ///< position at last CAM (ASN.1)
    HeadingValue_t              mLastCamHeading = HeadingValue_unavailable; ///< heading at last CAM (1/10°)
    double                      mLastCamSpeed_ms = 0.0;  ///< speed at last CAM [m/s]
    vanetza::Clock::time_point  mLastCamTime;             ///< simtime at last CAM
    bool                        mLastCamValid = false;    ///< false until first CAM sent
};

} // namespace veins
