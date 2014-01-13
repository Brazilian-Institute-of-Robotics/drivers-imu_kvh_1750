#ifndef FOG_KVH_1750_DRIVER_HPP
#define FOG_KVH_1750_DRIVER_HPP

#include <iodrivers_base/Driver.hpp>
#include <imu_kvh_1750/KVH1750Parser.hpp>
#include <imu_kvh_1750/KVH1750Types.hpp>

namespace imu_kvh_1750
{
  class Driver : public iodrivers_base::Driver , public KVH1750Parser
  {
    std::vector<uint8_t> buffer;
    int extractPacket (uint8_t const *buffer, size_t buffer_size) const;
    
    int mDesiredBaudrate;


  public:
    Driver();

      void open(std::string const& uri);

      /** Read available packets on the I/O */
      void read();
      fog_acceleration getAcceleration();
      fog_rotation_delta getRotationDelta();
      fog_simple_orientation getSimpleOrientation();
  };
}

#endif
