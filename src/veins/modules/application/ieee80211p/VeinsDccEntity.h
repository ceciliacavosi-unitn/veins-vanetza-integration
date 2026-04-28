#pragma once

#include <vanetza/dcc/entity.hpp>
#include <vanetza/dcc/transmit_rate_control.hpp>
#include <vanetza/dcc/channel_probe_processor.hpp>
#include <vanetza/dcc/channel_load.hpp>
#include <vanetza/common/clock.hpp>
#include <chrono>

namespace veins {

/**
 * Concrete implementation of Vanetza's TransmitRateControl interface.
 *
 * Implements ETSI EN 302 637-2 V1.4.1 Section 6.1.3 CAM generation rules:
 *
 * Every T_CheckCamGen (≤ 100 ms), a CAM shall be generated if:
 *
 *   Condition 1) time elapsed since last CAM ≥ T_GenCamMin (100 ms) AND
 *                at least one of:
 *                  - |Δheading|  > 4°
 *                  - |Δposition| > 4 m
 *                  - |Δspeed|    > 0.5 m/s
 *
 *   Condition 2) time elapsed since last CAM ≥ T_GenCam (default = 1000 ms)
 *                (timeout fallback — guarantees at least 1 CAM/sec)
 *
 * Limits:
 *   T_GenCamMin = 100 ms  (max rate: 10 Hz)
 *   T_GenCamMax = 1000 ms (min rate:  1 Hz)
 */
class VeinsTransmitRateControl : public vanetza::dcc::TransmitRateControl {
public:
    /** Returns the minimum delay before the next transmission (T_GenCamMin = 100 ms) */
    vanetza::Clock::duration delay(const vanetza::dcc::Transmission&) override;

    /**
     * Returns the current CAM generation interval.
     * - 100 ms  if a dynamics condition (Condition 1) was recently triggered
     * - 1000 ms otherwise (Condition 2 timeout fallback)
     */
    vanetza::Clock::duration interval(const vanetza::dcc::Transmission&) override;

    /** Called after each transmission to update internal state */
    void notify(const vanetza::dcc::Transmission&) override;

    /**
     * Called by VanetzaAdapter every T_CheckCamGen (≤ 100 ms).
     * Evaluates Condition 1 (dynamics) and Condition 2 (timeout).
     * Returns true if a CAM shall be generated now.
     *
     * @param deltaHeading_deg  |current heading - last CAM heading| in degrees
     * @param deltaPos_m        distance from last CAM position in metres
     * @param deltaSpeed_ms     |current speed - last CAM speed| in m/s
     * @param elapsed           time elapsed since last CAM generation
     */
    bool checkCamGeneration(double deltaHeading_deg,
                            double deltaPos_m,
                            double deltaSpeed_ms,
                            vanetza::Clock::duration elapsed);

private:
    // ETSI EN 302 637-2 §6.1.3 timing constants
    static constexpr auto T_GenCamMin = std::chrono::milliseconds(100);  ///< minimum CAM interval (10 Hz max)
    static constexpr auto T_GenCamMax = std::chrono::milliseconds(1000); ///< maximum CAM interval ( 1 Hz min)

    // ETSI EN 302 637-2 §6.1.3 dynamics thresholds
    static constexpr double HEADING_THRESHOLD_DEG = 4.0;  ///< heading change threshold [°]
    static constexpr double POSITION_THRESHOLD_M  = 4.0;  ///< position change threshold [m]
    static constexpr double SPEED_THRESHOLD_MS    = 0.5;  ///< speed change threshold [m/s]

    /**
     * Currently valid upper limit for CAM generation interval.
     * Set to elapsed time when Condition 1 fires, reset to T_GenCamMax
     * after N_GenCam consecutive high-frequency CAMs (ETSI §6.1.3).
     */
    vanetza::Clock::duration mTGenCam = T_GenCamMax;

    /**
     * Counter for consecutive CAMs triggered by Condition 1.
     * After N_GenCam = 3 consecutive CAMs, mTGenCam resets to T_GenCamMax.
     */
    int mNGenCamCount = 0;
    static constexpr int N_GenCam = 3; ///< max consecutive high-frequency CAMs (ETSI §6.1.3)
};

/**
 * Concrete implementation of Vanetza's ChannelProbeProcessor interface.
 * Receives channel busy ratio (CBR) measurements from the MAC layer.
 * Currently ignores measurements — can be extended to feed CBR into TRC.
 */
class VeinsChannelProbeProcessor : public vanetza::dcc::ChannelProbeProcessor {
public:
    /** Called with a new channel load measurement from the MAC layer */
    void indicate(vanetza::dcc::ChannelLoad) override;
};

/**
 * DCC Entity for Veins simulations.
 * Owns and exposes the TransmitRateControl and ChannelProbeProcessor
 * instances required by Vanetza's DCC subsystem.
 */
class VeinsDccEntity : public vanetza::dcc::Entity {
public:
    VeinsDccEntity() = default;
    ~VeinsDccEntity() = default;

    /** Returns the transmit rate controller */
    vanetza::dcc::TransmitRateControl& transmit_rate_control() override;

    /** Returns the channel probe processor */
    vanetza::dcc::ChannelProbeProcessor& channel_probe_processor() override;

    /**
     * Forwards CAM generation evaluation to the Transmit Rate Control (TRC).
     *
     * No logic here — the ETSI decision is implemented in TRC.
     */
    bool checkCamGeneration(bool headingChanged,
                            double deltaPos_m,
                            double deltaSpeed_ms,
                            vanetza::Clock::duration elapsed);

private:
    VeinsTransmitRateControl   mTrc; ///< Transmit rate controller instance
    VeinsChannelProbeProcessor mCpp; ///< Channel probe processor instance
};

} // namespace veins
