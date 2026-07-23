#include "angle_utils.h"

float Angle_Normalize180(float angle_deg)
{
    if (angle_deg != angle_deg) {
        return 0.0f;
    }

    while (angle_deg >= 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}
