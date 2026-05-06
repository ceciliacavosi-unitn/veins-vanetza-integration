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

using namespace omnetpp;

namespace veins {

class CamMessage;
class DenmMessage;

class VanetzaAdapter: public cSimpleModule
{
public:
    VanetzaAdapter();
    ~VanetzaAdapter(){}

    // ============================
    // TX helpers
    // ============================

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

    vanetza::dcc::Entity& getDccEntity();

    // ============================
    // DCC / CAM generation check
    // ============================

    bool computeAndCheckCamDeltas(const VeinsVehicleDataProvider& vdp);
    void notifyCamSent(const VeinsVehicleDataProvider& vdp);

    // DENM message cause code to string
    std::string denmCauseToString(int cause);

    void initialize() override;
    void handleMessage(omnetpp::cMessage* msg) override;

private:
    std::unique_ptr<VeinsDccEntity> mDccEntity;

    // --- Last CAM reference state (for ETSI §6.1.3 delta checks) ---
    ReferencePosition_t         mLastCamPos;
    HeadingValue_t              mLastCamHeading = HeadingValue_unavailable;
    double                      mLastCamSpeed_ms = 0.0;
    vanetza::Clock::time_point  mLastCamTime;
    bool                        mLastCamValid = false;
};

} // namespace veins
