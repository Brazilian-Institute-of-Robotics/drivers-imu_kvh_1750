#ifndef KVH_1750_PARSER_HPP
#define KVH_1750_PARSER_HPP

#include <stdint.h>
#include <base/time.h>
#include <base/eigen.h>
#include <boost/static_assert.hpp>
#include <vector>

#include <fog_kvh_1750/KVH1750Messages.hpp>
#include <fog_kvh_1750/KVH1750Types.hpp>

namespace fog_kvh_1750
{
    class KVH1750Parser
    {
    protected:
        int extractPacket(uint8_t const* buffer, size_t size, size_t max_size = 0) const;
        int getSizeOfMessage(uint16_t msg_id) const;
        void invalidateCellReadings();
        void parseMessage(uint8_t const* buffer, size_t size);

    public:

        Status status;
        FOGReading fog_reading;

        void parseEnsemble(uint8_t const* data, size_t size);
	fog_acceleration getAcceleration();
	fog_rotation_delta getRotationDelta();
	fog_simple_orientation getSimpleOrientation();


    };
}

#endif

