#include <imu_kvh_1750/Driver.hpp>
#include <sys/ioctl.h>
#include <termios.h>
#include <iostream>
#include <fstream>
#include <boost/crc.hpp>
#include <inttypes.h>
#include <arpa/inet.h>
#include <base/Logging.hpp>

using namespace imu_kvh_1750;

Driver::Driver()
    : iodrivers_base::Driver(1000000)
    , mDesiredBaudrate(921600)
{
    buffer.resize(1000000);
    imu.acc[0] = imu.acc[1] = imu.acc[2] = imu.gyro[0] = imu.gyro[1] = imu.gyro[2] = 0.0;
}

void Driver::open(std::string const& uri)
{
    openURI(uri);
}

void Driver::read()
{
    readPacket(&buffer[0], buffer.size());
    parseMessage(&buffer[4],buffer.size()-4);
}

int Driver::extractPacket (uint8_t const *buffer, size_t buffer_size) const
{
  // No valid packet start yet, discard this byte
  if(buffer[0] != 0xFE){
    //printf("Wrong start byte\n");
    return -1;
  }

  // wait for incoming bytes for full package length
  else if(buffer_size < 36){
    return 0;
  }

  // we have a valid header and 36 bytes
  else if(buffer[1] == 0x81 && buffer[2] == 0xFF && buffer[3] == 0x55){

    // check crc
    boost::crc_optimal<32,0x04C11DB7,0xFFFFFFFF,0,false,false> kvh_crc;
    kvh_crc.process_bytes(buffer,32);
    uint32_t checksum = htonl(* reinterpret_cast<const uint32_t*>(&buffer[32]));

    //check for right checksum
    if(checksum != kvh_crc.checksum()){
      LOG_WARN("WRONG CRC WARNING\n");
      return -buffer_size;
    }
    
    //check for valid status byte
    if(buffer[28] != 0x77){
      LOG_WARN("INVALID STATUS BYTE FROM KVH");
      return -buffer_size;
    }
    return buffer_size; // everything ok
  }
  return -buffer_size;
}

base::samples::IMUSensors Driver::getIMUReading(){
  return imu;
}

void Driver::parseMessage(uint8_t const* buffer, size_t size)
{

    int x_accel_int = buffer[15] | buffer[14] << 0x8 | buffer[13] << 0x10 | buffer[12] << 0x18;
    int y_accel_int = buffer[19] | buffer[18] << 0x8 | buffer[17] << 0x10 | buffer[16] << 0x18;
    int z_accel_int = buffer[23] | buffer[22] << 0x8 | buffer[21] << 0x10 | buffer[20] << 0x18;

    int x_delta_angle_int = buffer[3] | buffer[2] << 0x8 | buffer[1] << 0x10 | buffer[0] << 0x18;
    int y_delta_angle_int = buffer[7] | buffer[6] << 0x8 | buffer[5] << 0x10 | buffer[4] << 0x18;
    int z_delta_angle_int = buffer[11] | buffer[10] << 0x8 | buffer[9] << 0x10 | buffer[8] << 0x18;


    float x_accel = *(reinterpret_cast<float *>(&x_accel_int));
    float y_accel = *(reinterpret_cast<float *>(&y_accel_int));
    float z_accel = *(reinterpret_cast<float *>(&z_accel_int));

    float x_delta_angle = *(reinterpret_cast<float *>(&x_delta_angle_int));
    float y_delta_angle = *(reinterpret_cast<float *>(&y_delta_angle_int));
    float z_delta_angle = *(reinterpret_cast<float *>(&z_delta_angle_int));

    imu.time = base::Time::now();
    imu.acc[0] = x_accel;
    imu.acc[1] = y_accel;
    imu.acc[2] = z_accel;

    imu.gyro[0] = x_delta_angle;
    imu.gyro[1] = y_delta_angle;
    imu.gyro[2] = z_delta_angle;
}
