#pragma once

#include <memory>
#include <string>

#include "veins/modules/application/ieee80211p/VeinsVehicleDataProvider.h"

//#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"

// Vanetza
#include "vanetza/asn1/cam.hpp"
#include "vanetza/asn1/denm.hpp"
#include "vanetza/asn1/asn1c_wrapper.hpp"
#include "vanetza/common/clock.hpp"
#include "vanetza/units/angle.hpp"
#include "vanetza/units/velocity.hpp"
#include "vanetza/units/acceleration.hpp"
#include "vanetza/asn1/its/CauseCodeType.h"

// Veins messages
#include "veins/modules/messages/CamMessage_m.h"
#include "veins/modules/messages/DenmMessage_m.h"

namespace veins {

class CamMessage;
class DenmMessage;

class VanetzaAdapter : public cSimpleModule
{
public:
    VanetzaAdapter() = default;
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

    // DENM message cause code to string
    std::string denmCauseToString(int cause);

private:
    void setTimestamp(TimestampIts_t& ts, uint64_t ms);

};

} // namespace veins
