#ifndef FOG_KVH_1750_DRIVER_HPP
#define FOG_KVH_1750_DRIVER_HPP

#include <iodrivers_base/Driver.hpp>
#include <base/samples/IMUSensors.hpp>

namespace imu_kvh_1750
{
  class Driver : public iodrivers_base::Driver 
  {
    std::vector<uint8_t> buffer;
    int extractPacket (uint8_t const *buffer, size_t buffer_size) const;
    void parseMessage(uint8_t const* buffer, size_t size);
    
    int mDesiredBaudrate;

  private:
    base::samples::IMUSensors imu;

  public:
    Driver();

      void open(std::string const& uri);

      /** Read available packets on the I/O */
      void read();
      base::samples::IMUSensors getIMUReading();
  };
}

#endif
