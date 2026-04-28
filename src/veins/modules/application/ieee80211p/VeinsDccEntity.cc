#include "VeinsDccEntity.h"

namespace veins {

// ── VeinsTransmitRateControl ──────────────────────────────────────────────────

vanetza::Clock::duration VeinsTransmitRateControl::delay(const vanetza::dcc::Transmission&)
{
    // Minimum delay before next transmission: T_GenCamMin = 100 ms (ETSI §6.1.3)
    return T_GenCamMin;
}

vanetza::Clock::duration VeinsTransmitRateControl::interval(const vanetza::dcc::Transmission&)
{
    // Returns current CAM interval:
    //   T_GenCamMin (100 ms)  if Condition 1 was recently triggered
    //   T_GenCamMax (1000 ms) otherwise (Condition 2 timeout fallback)
    return mTGenCam;
}

void VeinsTransmitRateControl::notify(const vanetza::dcc::Transmission&)
{
    // No feedback processing at this stage
}

bool VeinsTransmitRateControl::checkCamGeneration(bool headingChanged,
                                                   double deltaPos_m,
                                                   double deltaSpeed_ms,
                                                   vanetza::Clock::duration elapsed)
{
    // --- Condition 1: dynamics-triggered CAM (ETSI EN 302 637-2 §6.1.3, rule 1) ---
    if (elapsed >= T_GenCamMin) {
        bool dynamicsChanged =
            headingChanged ||
            (deltaPos_m    > POSITION_THRESHOLD_M) ||
            (deltaSpeed_ms > SPEED_THRESHOLD_MS);

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

    // --- Condition 2: timeout fallback ---
    if (elapsed >= mTGenCam) {
        mNGenCamCount = 0;
        mTGenCam      = T_GenCamMax;
        return true;
    }

    return false;
}

// ── VeinsChannelProbeProcessor ────────────────────────────────────────────────

void VeinsChannelProbeProcessor::indicate(vanetza::dcc::ChannelLoad)
{
    // No channel load processing at this stage
    // Can be extended to feed CBR measurements into VeinsTransmitRateControl
}

// ── VeinsDccEntity ────────────────────────────────────────────────────────────

vanetza::dcc::TransmitRateControl& VeinsDccEntity::transmit_rate_control()
{
    // Returns the TRC used by Vanetza to query TX intervals for CAM scheduling
    return mTrc;
}

vanetza::dcc::ChannelProbeProcessor& VeinsDccEntity::channel_probe_processor()
{
    // Returns the CPP used by Vanetza to deliver CBR measurements from the MAC layer
    return mCpp;
}

bool VeinsDccEntity::checkCamGeneration(double deltaHeading_deg,
                                         double deltaPos_m,
                                         double deltaSpeed_ms,
                                         vanetza::Clock::duration elapsed)
{
    // Delegates to VeinsTransmitRateControl which implements ETSI §6.1.3 logic
    return mTrc.checkCamGeneration(deltaHeading_deg, deltaPos_m, deltaSpeed_ms, elapsed);
}

} // namespace veins
