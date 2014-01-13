#ifndef KVH_1750_MESSAGES_HPP
#define KVH_1750_MESSAGES_HPP

#include <stdint.h>
#include <base/time.h>
#include <base/eigen.h>
#include <vector>

namespace imu_kvh_1750
{
 struct Status
    {
        /*
        uint32_t seq;
        base::Time time;

        /*base::Quaterniond orientation;
        /** Standard deviation of orientation in yaw, pitch and roll */
        /*float stddev_orientation[3];

        float depth;
        float speed_of_sound;
        float salinity;
        float temperature;
        float pressure;
        float pressure_variance;

        uint8_t adc_channels[8];

        base::Time min_preping_wait;

        uint16_t self_test_result;
        uint32_t status_word; */
    };
 /** Bottom tracking information */
    struct FOGReading
    {
        /** Acquisition timestamp */
        base::Time time;

        /** Ranges to the bottom, in meters
         */
        //float range[4];
        /** Velocities. The reported information depends on the coordinate
         * transformation setting. See CellReading documentation for more
         * information
         */
        float velocity[4];
        /** Correlation at the bottom cell (between 0 and 1) */
        //float correlation[4];
        /** Magnitude in the evaluation filter, for each beam */
        //float evaluation[4];
        /** Ratio of good bottom tracking pings */
        //float good_ping_ratio[4];

        /** RSSI at the center of the bottom ping (in dB)
         */
        //float rssi[4];
    };
}

#endif

