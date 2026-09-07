#include "cansat_types.h"

const char *cansat_state_name(flight_state_t state)
{
    switch (state) {
    case CANSAT_STATE_STARTUP: return "STARTUP";
    case CANSAT_STATE_LAUNCH_PAD: return "LAUNCH_PAD";
    case CANSAT_STATE_ASCENT: return "ASCENT";
    case CANSAT_STATE_DESCENT: return "DESCENT";
    case CANSAT_STATE_MECH_RELEASED: return "MECH_RELEASED";
    case CANSAT_STATE_LANDED: return "LANDED";
    case CANSAT_STATE_ERROR: return "ERROR";
    default: return "ERROR";
    }
}

const char *cansat_mode_name(cansat_mode_t mode)
{
    return mode == CANSAT_MODE_SIMULATION ? "SIMULATION" : "FLIGHT";
}
