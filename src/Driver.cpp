#include <imu_kvh_1750/Driver.hpp>
#include <sys/ioctl.h>
#include <termios.h>
#include <iostream>
#include <fstream>

using namespace imu_kvh_1750;

Driver::Driver()
    : iodrivers_base::Driver(1000000)
    , mDesiredBaudrate(921600)
{
    buffer.resize(1000000);
}

void Driver::open(std::string const& uri)
{
    openURI(uri);
}

void Driver::read()
{
//    std::cout << "drivercpp: readPacket with buffer[0] = " << (int)buffer[0] << " and buffer.size = " << (int)buffer.size() << "\n" ;

  /*   std::cout << "first 40 buffer elements:";
    for (int i = 0; i < 40; ++i)
      std::cout << " " << (int)buffer[i];
*/
    int packet_size = readPacket(&buffer[0], buffer.size());
  //  std::cout << "drivercpp: readPacket returned packetsize = " << packet_size;

    if (packet_size)
        parseEnsemble(&buffer[0], packet_size);
}

int Driver::extractPacket (uint8_t const *buffer, size_t buffer_size) const
{
        // std::cout << iodrivers_base::Driver::printable_com(buffer, buffer_size) << std::endl;
        return KVH1750Parser::extractPacket(buffer, buffer_size);
}

fog_acceleration Driver::getAcceleration()
{
	return KVH1750Parser::getAcceleration();
}

fog_rotation_delta Driver::getRotationDelta()
{
	return KVH1750Parser::getRotationDelta();
}

fog_simple_orientation Driver::getSimpleOrientation()
{
	return KVH1750Parser::getSimpleOrientation();
}

