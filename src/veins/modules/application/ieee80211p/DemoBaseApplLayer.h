#pragma once

// Standard library
#include <map>
#include <memory>
#include <cmath>
#include <chrono>
#include <vector>
#include <cstdint>
#include <sstream>
#include <iomanip>
#include <set>

// Veins / OMNeT++ includes
#include "veins/base/modules/BaseApplLayer.h"
#include "veins/modules/utility/Consts80211p.h"
#include "veins/modules/messages/BaseFrame1609_4_m.h"
#include "veins/modules/messages/DemoServiceAdvertisement_m.h"
#include "veins/modules/messages/DemoSafetyMessage_m.h"
#include "veins/base/connectionManager/ChannelAccess.h"
#include "veins/modules/mac/ieee80211p/DemoBaseApplLayerToMac1609_4Interface.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"

// Application messages
#include "veins/modules/messages/CamMessage_m.h"
#include "veins/modules/messages/DenmMessage_m.h"

// Vanetza (ASN.1) includes used for CAM/DENM construction
#include "vanetza/asn1/cam.hpp"
#include "vanetza/asn1/denm.hpp"
#include "vanetza/asn1/asn1c_wrapper.hpp"
#include "vanetza/common/clock.hpp"
#include "vanetza/units/angle.hpp"
#include "vanetza/units/velocity.hpp"
#include "vanetza/units/acceleration.hpp"
#include "vanetza/asn1/its/CauseCodeType.h"
#include "vanetza/asn1/its/AccidentSubCauseCode.h"

namespace veins {

using veins::AnnotationManager;
using veins::AnnotationManagerAccess;
using veins::TraCICommandInterface;
using veins::TraCIMobility;
using veins::TraCIMobilityAccess;

/*
 * ============================================================
 * VeinsVehicleDataProvider
 * ============================================================
 */
class VEINS_API VeinsVehicleDataProvider {
public:
    explicit VeinsVehicleDataProvider(BaseMobility* mob);

    vanetza::units::GeoAngle latitude()  const;
    vanetza::units::GeoAngle longitude() const;
    vanetza::units::Velocity speed()     const;
    vanetza::units::Angle    heading()   const;
    uint32_t                 station_id() const;
    vanetza::Clock::time_point timestamp() const;

    vanetza::units::Acceleration acceleration() const;

    int get_event_cause() const;
    int get_event_subcause() const;

private:
    BaseMobility* mMobility;
};

/*
 * ============================================================
 * DemoBaseApplLayer
 * ============================================================
 */
class VEINS_API DemoBaseApplLayer : public BaseApplLayer {
public:
    ~DemoBaseApplLayer() override;

    void initialize(int stage) override;
    void finish() override;

    void receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) override;

    enum DemoApplMessageKinds {
        SEND_BEACON_EVT,
        SEND_WSA_EVT,
        SEND_CAM_EVT,
        SEND_DENM_EVT
    };

protected:
    // --------------------------------------------------------
    // Member Variables
    // --------------------------------------------------------

    /* Mobility & helper objects */
    BaseMobility* mobility = nullptr;
    TraCICommandInterface* traci = nullptr;
    TraCICommandInterface::Vehicle* traciVehicle = nullptr;
    AnnotationManager* annotations = nullptr;
    DemoBaseApplLayerToMac1609_4Interface* mac = nullptr;

    /* Bridge to Vanetza-style vehicle data */
    std::unique_ptr<VeinsVehicleDataProvider> mVehicleDataProvider;

    /* Parking state */
    bool isParked = false;

    /* DENM state tracking */
    std::set<int> activeDenmEvents;

    /* BSM settings */
    uint32_t beaconLengthBits = 0;
    uint32_t beaconUserPriority = 0;
    simtime_t beaconInterval = 0;
    bool sendBeacons = false;

    /* WSM settings */
    uint32_t dataLengthBits = 0;
    uint32_t dataUserPriority = 0;
    bool dataOnSch = false;

    /* DENM settings */
    uint32_t denmLengthBits = 0;
    uint32_t denmUserPriority = 0;
    uint16_t denmSequenceNumber;

    /* WSA settings */
    int currentOfferedServiceId = 0;
    std::string currentServiceDescription;
    Channel currentServiceChannel;
    simtime_t wsaInterval = 0;

    /* Runtime state */
    Coord curPosition;
    Coord curSpeed;
    LAddress::L2Type myId = 0;
    int mySCH = 0;

    /* Statistics counters */
    uint32_t generatedWSMs = 0;
    uint32_t generatedWSAs = 0;
    uint32_t generatedBSMs = 0;
    uint32_t generatedCAMs = 0;
    uint32_t generatedDENMs = 0;
    uint32_t receivedWSMs = 0;
    uint32_t receivedWSAs = 0;
    uint32_t receivedBSMs = 0;
    uint32_t receivedCAMs = 0;
    uint32_t receivedDENMs = 0;

    /* Self-message timers */
    cMessage* sendBeaconEvt = nullptr;
    cMessage* sendWSAEvt = nullptr;
    cMessage* sendCamEvt = nullptr;
    cMessage* sendDenmEvt = nullptr;

    /* DENM rate limiting */
    simtime_t denmMinInterval = 1.0;
    simtime_t lastDenmTime = SIMTIME_ZERO;
    int denmDefaultCause = 0;
    int denmDefaultSubcause = 0;

    // --------------------------------------------------------
    // Protected Methods
    // --------------------------------------------------------

    void handleLowerMsg(cMessage* msg) override;
    void handleSelfMsg(cMessage* msg) override;

    virtual void populateWSM(BaseFrame1609_4* wsm, LAddress::L2Type rcvId = LAddress::L2BROADCAST(), int serial = 0);

    virtual void onWSM(BaseFrame1609_4* wsm) {}
    virtual void onBSM(DemoSafetyMessage* bsm) {}
    virtual void onWSA(DemoServiceAdvertisment* wsa) {}
    virtual void onCAM(CamMessage* cam);
    virtual void onDENM(DenmMessage* denm);

    virtual void triggerDenm(CauseCodeType_t eventCause, int cause = -1, int subcause = -1, const Coord* eventPos = nullptr, bool sendImmediate = true);

    virtual void handlePositionUpdate(cObject* obj);
    virtual void handleParkingUpdate(cObject* obj);

    virtual void startService(Channel channel, int serviceId, std::string serviceDescription);
    virtual void stopService();

    virtual simtime_t computeAsynchronousSendingTime(simtime_t interval, ChannelType chantype);

    virtual void sendDown(cMessage* msg);
    virtual void sendDelayedDown(cMessage* msg, simtime_t delay);
    virtual void sendDenmNow(CauseCodeType_t eventCause, int cause, int subcause, const Coord& eventPos);
    virtual void checkAndTrackPacket(cMessage* msg);
};

} // namespace veins
