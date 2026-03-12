#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"

using namespace veins;

// ============================================================
//  VeinsVehicleDataProvider  —  bridge Veins <-> Vanetza
// ============================================================

VeinsVehicleDataProvider::VeinsVehicleDataProvider(TraCIMobility* mob)
    : mMobility(mob) {}

/* Latitude/Longitude in gradi (GeoAngle expects degrees) */
vanetza::units::GeoAngle VeinsVehicleDataProvider::latitude() const {
    if (!mMobility) return 0.0 * boost::units::degree::degree;
    double lat_m = mMobility->getPositionAt(simTime()).y;
    // Convert meters -> degrees using approximate scale (1 deg ~ 111000 m)
    double deg = 45.0 + (lat_m / 111000.0);
    return deg * boost::units::degree::degree;
}

vanetza::units::GeoAngle VeinsVehicleDataProvider::longitude() const {
    if (!mMobility) return 0.0 * boost::units::degree::degree;
    double lon_m = mMobility->getPositionAt(simTime()).x;
    double deg = 9.0 + (lon_m / 111000.0);
    return deg * boost::units::degree::degree;
}

/* Speed in m/s as vanetza::units::Velocity (SI) */
vanetza::units::Velocity VeinsVehicleDataProvider::speed() const {
    if (!mMobility) return 0.0 * boost::units::si::meter_per_second;
    double spd = mMobility->getSpeed();
    return spd * boost::units::si::meter_per_second;
}

/* Heading must return vanetza::units::Angle (radians) */
vanetza::units::Angle VeinsVehicleDataProvider::heading() const {
    if (!mMobility) return 0.0 * boost::units::si::radian;

    // Use Veins Heading getter that returns radians
    double rad = mMobility->getHeading().getRad();

    // Normalize to [0, 2*pi)
    const double two_pi = 2.0 * M_PI;
    double normalized_rad = std::fmod(std::fmod(rad, two_pi) + two_pi, two_pi);

    return normalized_rad * boost::units::si::radian;
}

uint32_t VeinsVehicleDataProvider::station_id() const {
    if (!mMobility) return 0u;
    return static_cast<uint32_t>(mMobility->getId());
}

vanetza::Clock::time_point VeinsVehicleDataProvider::timestamp() const {
    using namespace std::chrono;
    auto ms = duration_cast<milliseconds>(duration<double>(simTime().dbl()));
    return vanetza::Clock::time_point(ms);
}

// ============================================================
//  Helpers CAM (Vanetza ETSI EN 302 637-2)
// ============================================================

/**
 * Costruisce un CAM ETSI-compliant a partire dal VehicleDataProvider.
 * Popola: header, BasicContainer, HighFrequencyContainer.
 *
 * Nota: si assume che vdp rappresenti un provider valido
 * (chiamare solo quando mVehicleDataProvider non è nullptr).
 */
static vanetza::asn1::Cam buildCam(const VeinsVehicleDataProvider& vdp)
{
    vanetza::asn1::Cam cam;

    // --- ITS PDU Header ---
    cam->header.messageID       = ItsPduHeader__messageID_cam;
    cam->header.protocolVersion = 2;
    cam->header.stationID       = vdp.station_id();

    // --- GenerationDeltaTime (modulo 65536 ms) ---
    uint32_t now_ms = static_cast<uint32_t>(std::lround(simTime().dbl() * 1000.0)) % 65536;
    cam->cam.generationDeltaTime = static_cast<GenerationDeltaTime_t>(now_ms);

    // --- BasicContainer ---
    auto &basic = cam->cam.camParameters.basicContainer;
    basic.stationType = StationType_passengerCar;

    // Posizione ETSI: 1e-7 gradi
    // Assumiamo vdp.latitude() e longitude() ritornino quantita' in gradi (value() -> numeric degrees)
    double lat_deg = vdp.latitude().value();   // numeric degrees
    double lon_deg = vdp.longitude().value();

    basic.referencePosition.latitude  = static_cast<Latitude_t>(std::lround(lat_deg * 1e7));
    basic.referencePosition.longitude = static_cast<Longitude_t>(std::lround(lon_deg * 1e7));

    basic.referencePosition.positionConfidenceEllipse.semiMajorConfidence =
        SemiAxisLength_unavailable;
    basic.referencePosition.positionConfidenceEllipse.semiMinorConfidence =
        SemiAxisLength_unavailable;
    basic.referencePosition.positionConfidenceEllipse.semiMajorOrientation =
        HeadingValue_unavailable;
    basic.referencePosition.altitude.altitudeValue =
        AltitudeValue_unavailable;
    basic.referencePosition.altitude.altitudeConfidence =
        AltitudeConfidence_unavailable;

    // --- HighFrequencyContainer ---
    auto &hfc = cam->cam.camParameters.highFrequencyContainer;
    hfc.present = HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;
    auto &bvchf = hfc.choice.basicVehicleContainerHighFrequency;

    // Speed: ETSI uses cm/s. Assume vdp.speed().value() returns m/s
    double speed_m_s = vdp.speed().value();
    double spd_cm_s = speed_m_s * 100.0;
    if (!std::isfinite(spd_cm_s) || spd_cm_s < 0.0) {
        bvchf.speed.speedValue = SpeedValue_unavailable;
    } else if (spd_cm_s >= static_cast<double>(SpeedValue_unavailable)) {
        bvchf.speed.speedValue = SpeedValue_unavailable;
    } else {
        bvchf.speed.speedValue = static_cast<SpeedValue_t>(std::lround(spd_cm_s));
    }
    bvchf.speed.speedConfidence = SpeedConfidence_unavailable;

    // Heading: convert from radians -> degrees if needed
    // Here convert vdp.heading() numeric value (radians) to degrees:
    double heading_rad = vdp.heading().value(); // assume rad
    double heading_deg = heading_rad * (180.0 / M_PI);
    double heading_tenths = std::fmod(std::fmod(heading_deg, 360.0) + 360.0, 360.0) * 10.0; // decimi di grado

    if (!std::isfinite(heading_tenths)) {
        bvchf.heading.headingValue = HeadingValue_unavailable;
    } else {
        if (heading_tenths < 0.0) heading_tenths = 0.0;
        if (heading_tenths > 3600.0) heading_tenths = 3600.0;
        bvchf.heading.headingValue = static_cast<HeadingValue_t>(std::lround(heading_tenths));
    }
    bvchf.heading.headingConfidence = HeadingConfidence_unavailable;

    // altri campi obbligatori
    bvchf.driveDirection = DriveDirection_forward;
    bvchf.vehicleLength.vehicleLengthValue = VehicleLengthValue_unavailable;
    bvchf.vehicleLength.vehicleLengthConfidenceIndication = VehicleLengthConfidenceIndication_unavailable;
    bvchf.vehicleWidth = VehicleWidth_unavailable;
    bvchf.longitudinalAcceleration.longitudinalAccelerationValue = LongitudinalAccelerationValue_unavailable;
    bvchf.longitudinalAcceleration.longitudinalAccelerationConfidence = AccelerationConfidence_unavailable;
    bvchf.curvature.curvatureValue = CurvatureValue_unavailable;
    bvchf.curvature.curvatureConfidence = CurvatureConfidence_unavailable;
    bvchf.curvatureCalculationMode = CurvatureCalculationMode_unavailable;
    bvchf.yawRate.yawRateValue = YawRateValue_unavailable;
    bvchf.yawRate.yawRateConfidence = YawRateConfidence_unavailable;

    return cam;
}

// ============================================================
//  DemoBaseApplLayer
// ============================================================

void DemoBaseApplLayer::initialize(int stage)
{
    BaseApplLayer::initialize(stage);

    if (stage == 0) {

        // initialize pointers to other modules
        if (FindModule<TraCIMobility*>::findSubModule(getParentModule())) {
            mobility = TraCIMobilityAccess().get(getParentModule());
            traci = mobility->getCommandInterface();
            traciVehicle = mobility->getVehicleCommandInterface();
        }
        else {
            traci = nullptr;
            mobility = nullptr;
            traciVehicle = nullptr;
        }

        annotations = AnnotationManagerAccess().getIfExists();
        ASSERT(annotations);

        mac = FindModule<DemoBaseApplLayerToMac1609_4Interface*>::findSubModule(getParentModule());
        ASSERT(mac);

        // read parameters
        headerLength      = par("headerLength");
        sendBeacons       = par("sendBeacons").boolValue();
        beaconLengthBits  = par("beaconLengthBits");
        beaconUserPriority = par("beaconUserPriority");
        beaconInterval    = par("beaconInterval");

        dataLengthBits    = par("dataLengthBits");
        dataOnSch         = par("dataOnSch").boolValue();
        dataUserPriority  = par("dataUserPriority");

        wsaInterval            = par("wsaInterval").doubleValue();
        currentOfferedServiceId = -1;

        isParked = false;

        findHost()->subscribe(BaseMobility::mobilityStateChangedSignal, this);
        findHost()->subscribe(TraCIMobility::parkingStateChangedSignal, this);

        sendBeaconEvt = new cMessage("beacon evt", SEND_BEACON_EVT);
        sendWSAEvt    = new cMessage("wsa evt",    SEND_WSA_EVT);
        sendCamEvt    = new cMessage("cam evt",    SEND_CAM_EVT);

        generatedBSMs = 0;
        generatedWSAs = 0;
        generatedWSMs = 0;
        generatedCAMs = 0;
        receivedBSMs  = 0;
        receivedWSAs  = 0;
        receivedWSMs  = 0;
        receivedCAMs  = 0;

        // --- Inizializza VehicleDataProvider Vanetza (se mobility già disponibile) ---
        if (mobility) {
            mVehicleDataProvider = std::make_unique<VeinsVehicleDataProvider>(mobility);
        }
    }
    else if (stage == 1) {

        // store MAC address for quick access
        myId = mac->getMACAddress();

        if (dataOnSch == true && !mac->isChannelSwitchingActive()) {
            dataOnSch = false;
            EV_ERROR << "App wants to send data on SCH but MAC doesn't use any SCH. "
                        "Sending all data on CCH" << std::endl;
        }

        simtime_t firstBeacon = simTime();

        if (par("avoidBeaconSynchronization").boolValue() == true) {

            simtime_t randomOffset = dblrand() * beaconInterval;
            firstBeacon = simTime() + randomOffset;

            if (mac->isChannelSwitchingActive() == true) {
                if (beaconInterval.raw() % (mac->getSwitchingInterval().raw() * 2)) {
                    EV_ERROR << "The beacon interval (" << beaconInterval
                             << ") is smaller than or not a multiple of one synchronization interval ("
                             << 2 * mac->getSwitchingInterval()
                             << "). This means that beacons are generated during SCH intervals"
                             << std::endl;
                }
                firstBeacon = computeAsynchronousSendingTime(beaconInterval, ChannelType::control);
            }

            if (sendBeacons) {
                scheduleAt(firstBeacon, sendBeaconEvt);
            }
        }

        // Se mobility non era disponibile in stage 0, proviamo a crearne il provider ora
        if (!mVehicleDataProvider && mobility) {
            mVehicleDataProvider = std::make_unique<VeinsVehicleDataProvider>(mobility);
        }

        // CAM ETSI: primo invio dopo 1 s (poi ogni 100 ms) — scheduliamo solo se abbiamo un provider
        if (mVehicleDataProvider) {
            scheduleAt(simTime() + 1.0, sendCamEvt);
        } else {
            EV_INFO << "No VeinsVehicleDataProvider available for this node — CAM will not be scheduled\n";
        }
    }
}

// ------------------------------------------------------------

simtime_t DemoBaseApplLayer::computeAsynchronousSendingTime(simtime_t interval, ChannelType chan)
{
    simtime_t randomOffset    = dblrand() * interval;
    simtime_t firstEvent;
    simtime_t switchingInterval = mac->getSwitchingInterval(); // usually 0.050 s
    simtime_t nextCCH;

    if (mac->isCurrentChannelCCH()) {
        nextCCH = simTime() - SimTime().setRaw(simTime().raw() % switchingInterval.raw())
                  + switchingInterval * 2;
    }
    else {
        nextCCH = simTime() - SimTime().setRaw(simTime().raw() % switchingInterval.raw())
                  + switchingInterval;
    }

    firstEvent = nextCCH + randomOffset;

    if (firstEvent.raw() % (2 * switchingInterval.raw()) > switchingInterval.raw()) {
        if (chan == ChannelType::control) firstEvent -= switchingInterval;
    }
    else {
        if (chan == ChannelType::service) firstEvent += switchingInterval;
    }

    return firstEvent;
}

// ------------------------------------------------------------

void DemoBaseApplLayer::populateWSM(BaseFrame1609_4* wsm, LAddress::L2Type rcvId, int serial)
{
    wsm->setRecipientAddress(rcvId);
    wsm->setBitLength(headerLength);

    if (DemoSafetyMessage* bsm = dynamic_cast<DemoSafetyMessage*>(wsm)) {
        bsm->setSenderPos(curPosition);
        bsm->setSenderSpeed(curSpeed);
        bsm->setPsid(-1);
        bsm->setChannelNumber(static_cast<int>(Channel::cch));
        bsm->addBitLength(beaconLengthBits);
        wsm->setUserPriority(beaconUserPriority);
    }
    else if (DemoServiceAdvertisment* wsa = dynamic_cast<DemoServiceAdvertisment*>(wsm)) {
        wsa->setChannelNumber(static_cast<int>(Channel::cch));
        wsa->setTargetChannel(static_cast<int>(currentServiceChannel));
        wsa->setPsid(currentOfferedServiceId);
        wsa->setServiceDescription(currentServiceDescription.c_str());
    }
    else if (CamMessage* cam = dynamic_cast<CamMessage*>(wsm)) {
        // Proteggiamo la generazione del CAM: richiediamo che esista il provider
        if (mVehicleDataProvider) {
            auto vanetzaCam = buildCam(*mVehicleDataProvider);
            vanetza::ByteBuffer buffer = vanetzaCam.encode();

            // Salva il payload nel messaggio OMNeT++
            cam->setVanetzaPayloadArraySize(buffer.size());
            for (size_t i = 0; i < buffer.size(); ++i) {
                cam->setVanetzaPayload(i, buffer[i]);
            }

            cam->setByteLength(buffer.size());
        } else {
            // Nessun provider: imposta payload vuoto e segnala
            EV_WARN << "populateWSM: no VeinsVehicleDataProvider available — CAM payload vuoto\n";
            cam->setVanetzaPayloadArraySize(0);
            cam->setByteLength(0);
        }
    }
    else {
        if (dataOnSch)
            wsm->setChannelNumber(static_cast<int>(Channel::sch1));
        else
            wsm->setChannelNumber(static_cast<int>(Channel::cch));
        wsm->addBitLength(dataLengthBits);
        wsm->setUserPriority(dataUserPriority);
    }
}

// ------------------------------------------------------------

void DemoBaseApplLayer::receiveSignal(cComponent* source, simsignal_t signalID,
                                      cObject* obj, cObject* details)
{
    Enter_Method_Silent();
    if (signalID == BaseMobility::mobilityStateChangedSignal) {
        handlePositionUpdate(obj);
    }
    else if (signalID == TraCIMobility::parkingStateChangedSignal) {
        handleParkingUpdate(obj);
    }
}

void DemoBaseApplLayer::onCAM(CamMessage* camMsg)
{
    size_t n = camMsg->getVanetzaPayloadArraySize();
    vanetza::ByteBuffer buffer(n);
    for (size_t i = 0; i < n; ++i) {
        buffer[i] = static_cast<uint8_t>(camMsg->getVanetzaPayload(i));
    }

    try {
        vanetza::asn1::Cam cam;
            cam.decode(buffer);

       uint32_t id = cam->header.stationID;
        double lat = static_cast<double>(cam->cam.camParameters.basicContainer.referencePosition.latitude) / 1e7;
        double lon = static_cast<double>(cam->cam.camParameters.basicContainer.referencePosition.longitude) / 1e7;

        EV_INFO << ">>> RICEVUTO CAM DA STATION ID: " << id << " | POS: " << lat << ", " << lon << " <<<" << endl;

    } catch (const std::exception& e) {
        EV_ERROR << "Errore decodifica Vanetza: " << e.what() << endl;
    }
}

void DemoBaseApplLayer::handlePositionUpdate(cObject* obj)
{
    // 1. Chiamata standard per aggiornare posizione/velocità
    ChannelMobilityPtrType const mobility = check_and_cast<ChannelMobilityPtrType>(obj);
    curPosition = mobility->getPositionAt(simTime());
    curSpeed    = mobility->getCurrentSpeed();

    // 2. CONTROLLO VANETZA: Se l'auto è appena apparsa, crea il provider e il timer
    if (!mVehicleDataProvider && this->mobility) {
        mVehicleDataProvider = std::make_unique<VeinsVehicleDataProvider>(this->mobility);

        // Se il timer non è ancora stato schedulato, facciamolo ora!
        if (!sendCamEvt->isScheduled()) {
            scheduleAt(simTime() + 1.0, sendCamEvt);
            EV_INFO << "Auto apparsa in SUMO: Provider creato e CAM schedulato per " << getFullPath() << endl;
        }
    }
}

void DemoBaseApplLayer::handleParkingUpdate(cObject* obj)
{
    isParked = mobility->getParkingState();
}

// ------------------------------------------------------------

void DemoBaseApplLayer::handleLowerMsg(cMessage* msg)
{
    BaseFrame1609_4* wsm = dynamic_cast<BaseFrame1609_4*>(msg);
    ASSERT(wsm);

    if (DemoSafetyMessage* bsm = dynamic_cast<DemoSafetyMessage*>(wsm)) {
        receivedBSMs++;
        onBSM(bsm);
    }
    else if (DemoServiceAdvertisment* wsa = dynamic_cast<DemoServiceAdvertisment*>(wsm)) {
        receivedWSAs++;
        onWSA(wsa);
    }
    else if (CamMessage* cam = dynamic_cast<CamMessage*>(wsm)) {
        receivedCAMs++;
        onCAM(cam);
    }
    else {
        receivedWSMs++;
        onWSM(wsm);
    }

    delete msg;
}

// ------------------------------------------------------------

void DemoBaseApplLayer::handleSelfMsg(cMessage* msg)
{
    switch (msg->getKind()) {

    case SEND_BEACON_EVT: {
        DemoSafetyMessage* bsm = new DemoSafetyMessage();
        populateWSM(bsm);
        sendDown(bsm);
        scheduleAt(simTime() + beaconInterval, sendBeaconEvt);
        break;
    }

    case SEND_WSA_EVT: {
        DemoServiceAdvertisment* wsa = new DemoServiceAdvertisment();
        populateWSM(wsa);
        sendDown(wsa);
        scheduleAt(simTime() + wsaInterval, sendWSAEvt);
        break;
    }

    case SEND_CAM_EVT: {
        // Costruisci e invia il CAM ETSI tramite Vanetza
        // Se non abbiamo ancora il provider (mobility non inizializzata), salta l'invio
        if (!mVehicleDataProvider) {
            EV_WARN << "SEND_CAM_EVT: VeinsVehicleDataProvider not available yet - skipping CAM and retrying\n";
            // riproviamo dopo 100ms (non bloccante)
            scheduleAt(simTime() + 0.1, sendCamEvt);
            break;
        }

        CamMessage* cam = new CamMessage();
        populateWSM(cam);
        sendDown(cam);

        // ETSI EN 302 637-2: intervallo CAM tra 100 ms e 1 s
        // Qui usiamo 100 ms (T_GenCam_Dcc minimo)
        scheduleAt(simTime() + 0.1, sendCamEvt);
        break;
    }

    default:
        if (msg) EV_WARN << "APP: Error: Got Self Message of unknown kind! Name: "
                         << msg->getName() << endl;
        break;
    }
}

// ------------------------------------------------------------

void DemoBaseApplLayer::finish()
{
    recordScalar("generatedWSMs", generatedWSMs);
    recordScalar("receivedWSMs",  receivedWSMs);

    recordScalar("generatedBSMs", generatedBSMs);
    recordScalar("receivedBSMs",  receivedBSMs);

    recordScalar("generatedWSAs", generatedWSAs);
    recordScalar("receivedWSAs",  receivedWSAs);

    recordScalar("generatedCAMs", generatedCAMs);
    recordScalar("receivedCAMs",  receivedCAMs);
}

DemoBaseApplLayer::~DemoBaseApplLayer()
{
    cancelAndDelete(sendBeaconEvt);
    cancelAndDelete(sendWSAEvt);
    cancelAndDelete(sendCamEvt);
    findHost()->unsubscribe(BaseMobility::mobilityStateChangedSignal, this);
}

// ------------------------------------------------------------

void DemoBaseApplLayer::startService(Channel channel, int serviceId,
                                     std::string serviceDescription)
{
    if (sendWSAEvt->isScheduled()) {
        throw cRuntimeError("Starting service although another service was already started");
    }

    mac->changeServiceChannel(channel);
    currentOfferedServiceId  = serviceId;
    currentServiceChannel    = channel;
    currentServiceDescription = serviceDescription;

    simtime_t wsaTime = computeAsynchronousSendingTime(wsaInterval, ChannelType::control);
    scheduleAt(wsaTime, sendWSAEvt);
}

void DemoBaseApplLayer::stopService()
{
    cancelEvent(sendWSAEvt);
    currentOfferedServiceId = -1;
}

// ------------------------------------------------------------

void DemoBaseApplLayer::sendDown(cMessage* msg)
{
    checkAndTrackPacket(msg);
    BaseApplLayer::sendDown(msg);
}

void DemoBaseApplLayer::sendDelayedDown(cMessage* msg, simtime_t delay)
{
    checkAndTrackPacket(msg);
    BaseApplLayer::sendDelayedDown(msg, delay);
}

void DemoBaseApplLayer::checkAndTrackPacket(cMessage* msg)
{
    if (dynamic_cast<DemoSafetyMessage*>(msg)) {
        EV_TRACE << "sending down a BSM" << std::endl;
        generatedBSMs++;
    }
    else if (dynamic_cast<DemoServiceAdvertisment*>(msg)) {
        EV_TRACE << "sending down a WSA" << std::endl;
        generatedWSAs++;
    }
    else if (dynamic_cast<CamMessage*>(msg)) {
        EV_TRACE << "sending down a CAM (Vanetza ETSI)" << std::endl;
        generatedCAMs++;
    }
    else if (dynamic_cast<BaseFrame1609_4*>(msg)) {
        EV_TRACE << "sending down a WSM" << std::endl;
        generatedWSMs++;
    }
}
