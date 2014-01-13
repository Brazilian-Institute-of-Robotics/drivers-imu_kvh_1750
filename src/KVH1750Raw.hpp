#ifndef KVH_1750_RAW_HPP
#define KVH_1750_RAW_HPP

#include <stdint.h>
#include <boost/static_assert.hpp>
#include <vector>

namespace imu_kvh_1750
{
    namespace raw
    {
        struct Header
        {
            enum MSG_ID { ID0 = 0xfe, ID1 = 0x81, ID2 = 0xff, ID3 = 0x55 };
            /*uint8_t id;
            uint8_t data_source_id;
            uint16_t size;
            uint8_t  spare;
            uint8_t  msg_count;
            uint16_t offsets[0];*/
        } __attribute__((packed));

        struct VelocityMessage
        {
            enum MSG_ID { ID = 0x0100 };
            int16_t id;
//            CellVelocity velocities[0];
        } __attribute__((packed));

    }
}

#endif
