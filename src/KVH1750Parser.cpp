#include <imu_kvh_1750/KVH1750Parser.hpp>
#include <imu_kvh_1750/KVH1750Raw.hpp>
#include <imu_kvh_1750/KVH1750Types.hpp>
#include <endian.h>
#include <stdexcept>
#include <base/float.h>
#include <base/time.h>

#include <boost/lexical_cast.hpp>
#include <boost/crc.hpp>
#include <string>
using boost::lexical_cast;
using std::string;

using namespace imu_kvh_1750;

float x_angle = 0;
float y_angle = 0;
float z_angle = 0;

fog_acceleration acceleration;
fog_rotation_delta rotation_delta;
fog_simple_orientation simple_orientation;

int KVH1750Parser::extractPacket(uint8_t const* buffer, size_t size, size_t max_size) const
{

    // Look for the first "thing" that looks like a valid header start
    size_t packet_start;
    for (packet_start = 0; packet_start < size; ++packet_start)
    {
//        std::cout << "kvh parse cpp: check packet start byte vs ID : " << (int)buffer[packet_start] << " vs " << raw::Header::ID0 << std::endl;
        if (buffer[packet_start] == raw::Header::ID0)
	{
//	    std::cout << "found header ID! - breaking for loop...\n";

  /*       std::cout << "buffer at packet_start: " << (int)buffer[packet_start] << " " << (int)buffer[packet_start+1] << " " <<
         (int)buffer[packet_start+2] << " " <<(int)buffer[packet_start+3] << "\n";
*/

            break;
	}
    }

    //std::cout << "packet start = " << packet_start << " size = " << size << " max_size = " << max_size;

    if (packet_start == size)
    {
      //  std::cout << " no packet start in buffer, discard everything";
        return -size;
    }
    else if (packet_start)
    {
       //std::cout << " realign the IODriver buffer to the start of the candidate packet, returning -" << packet_start;
       return -packet_start;
    }
    else if (size > 1 && buffer[1] != raw::Header::ID1)
    {
        //std::cout << " not actually a packet. Drop the first two bytes and let IODriver call us back";
        return -2; 
    }
    else if (size > 2 && buffer[2] != raw::Header::ID2)
    {
        //std::cout << " not actually a packet. Drop the first 3 bytes and let IODriver call us back";
        return -3; 
    }
    else if (size > 3 && buffer[3] != raw::Header::ID3)
    {
        //std::cout << " not actually a packet. Drop the first 4 bytes and let IODriver call us back";
        return -4; 
    }
    else if (size < sizeof(raw::Header))
    {
        //std::cout <<" cannot parse the rest of the header yet ... wait for new data";
        return 0;
    }
 raw::Header const& header = *reinterpret_cast<raw::Header const*>(buffer);
    // This is the size EXCLUDING CHECKSUM
    uint16_t ensemble_size = 32; //le16toh(header.size);
    uint16_t total_size = ensemble_size + 4;
    if (max_size && max_size < total_size)
    {
        //std::cout << "max_size " << max_size << " < " << " total_size " << total_size;

        // Assume that this packet is not valid as it has a size too big. Drop
        // the first two bytes, and let IODriver call us back to parse the rest
        // of the buffer
        return -2;
    }
    else if (size < total_size)
    {
        //std::cout << "size " << size << " < " << " total_size " << total_size << ", wait for more data";
        // Have to wait for new data (we don't have a full packet yet)
        return 0;
    }

    uint16_t checksum = 0;
    /*std::cout << "parser extractPacket: packet elements (buffer 0 to ens_size+4):";
    for (int i = 0; i < ensemble_size +4; ++i)
    {  std::cout << " " << (int)buffer[i];
       checksum += buffer[i];
    }
    std::cout << std::endl;
*/

   /* typedef boost::crc_optimal<32, 0x04C11DB7, 0xFFFFFFFF, 0x0, false, false>;
    crc_32_kvh_type;


    crc_32_kvh_type  result;

    result.process_bytes( buffer, ensemble_size );
    std::cout << "crc: " << result.checksum() << std::endl;

    // f5 e7 5b c3

    std::cout << "checksum = " << checksum;

    uint16_t msg_checksum = le16toh(*reinterpret_cast<uint16_t const*>(buffer + ensemble_size));

    std::cout << "msg checksum = " << msg_checksum;
*/

  /*  if (checksum != msg_checksum)
    {
        // Not a valid message. Drop the message IDs and let IODriver call us
        // back to find the start of the actual packet
        return -2;
    }
*/

/*   if (sizeof(raw::Header) + header.msg_count * 2 > ensemble_size)
        return -2;
    uint32_t offsets[256];
    for (int i = 0; i < header.msg_count; ++i)
        offsets[i] = le16toh(header.offsets[i]);

    // Validate sizes
    uint32_t expected_offset = 0;
    for (int i = 0; i < header.msg_count; ++i)
    {
        if (expected_offset != 0 && offsets[i] != expected_offset)
            return -2;

        uint32_t msg_id   = le16toh(*reinterpret_cast<uint16_t const*>(buffer + offsets[i]));
        uint32_t msg_size = getSizeOfMessage(msg_id);
        if (msg_size != 0)
            expected_offset = offsets[i] + msg_size;
    }*/
//    std::cout << "returning total size = " << total_size;
    return total_size;
}

int KVH1750Parser::getSizeOfMessage(uint16_t msg_id) const
{
    return 0;
}

void KVH1750Parser::parseMessage(uint8_t const* buffer, size_t size)
{
//    std::cout << "parseMessage with buffer " << (int)buffer[0] <<" "<< (int)buffer[1] << " ... " << (int)buffer[size-2] << " " << (int)buffer[size-1] << "\n";

//    std::cout << "z accel bytes: " << (int)buffer[20] << " " <<(int)buffer[21] << " " <<(int)buffer[22] << " " <<(int)buffer[23] << "\n";

    int x_accel_int = buffer[15] | buffer[14] << 0x8 | buffer[13] << 0x10 | buffer[12] << 0x18;
    int y_accel_int = buffer[19] | buffer[18] << 0x8 | buffer[17] << 0x10 | buffer[16] << 0x18;
    int z_accel_int = buffer[23] | buffer[22] << 0x8 | buffer[21] << 0x10 | buffer[20] << 0x18;

    int x_delta_angle_int = buffer[3] | buffer[2] << 0x8 | buffer[1] << 0x10 | buffer[0] << 0x18;
    int y_delta_angle_int = buffer[7] | buffer[6] << 0x8 | buffer[5] << 0x10 | buffer[4] << 0x18;
    int z_delta_angle_int = buffer[11] | buffer[10] << 0x8 | buffer[9] << 0x10 | buffer[8] << 0x18;

  /*  std::cout << "x delta angle bytes: " << (int)buffer[0] << " " <<(int)buffer[1] << " " <<(int)buffer[2] << " " <<(int)buffer[3] << "\n";
    std::cout << "y delta angle bytes: " << (int)buffer[4] << " " <<(int)buffer[5] << " " <<(int)buffer[6] << " " <<(int)buffer[7] << "\n";
    std::cout << "z delta angle bytes: " << (int)buffer[8] << " " <<(int)buffer[9] << " " <<(int)buffer[10] << " " <<(int)buffer[11] << "\n";


    std::cout << "x_delta_angle_int = " << x_delta_angle_int << "\n";
    std::cout << "y_delta_angle_int = " << y_delta_angle_int << "\n";
    std::cout << "z_delta_angle_int = " << z_delta_angle_int << "\n";
*/

    float x_accel = *(reinterpret_cast<float *>(&x_accel_int));
    float y_accel = *(reinterpret_cast<float *>(&y_accel_int));
    float z_accel = *(reinterpret_cast<float *>(&z_accel_int));

    float x_delta_angle = *(reinterpret_cast<float *>(&x_delta_angle_int));
    float y_delta_angle = *(reinterpret_cast<float *>(&y_delta_angle_int));
    float z_delta_angle = *(reinterpret_cast<float *>(&z_delta_angle_int));

  /*  std::cout << "x_delta_angle = " << x_delta_angle << "\n";
    std::cout << "y_delta_angle = " << y_delta_angle << "\n";
    std::cout << "z_delta_angle = " << z_delta_angle << "\n";
*/


/*    std::cout << "x_accel = " << x_accel << "\n";
    std::cout << "y_accel = " << y_accel << "\n";
    std::cout << "z_accel = " << z_accel << "\n";
*/


  /*  std::cout << "x_delta_angle = " << x_delta_angle << "\n";
    std::cout << "y_delta_angle = " << y_delta_angle << "\n";
    std::cout << "z_delta_angle = " << z_delta_angle << "\n";
*/
    x_angle += x_delta_angle;
    y_angle += y_delta_angle;
    z_angle += z_delta_angle;

  acceleration.time = base::Time::now();
  acceleration.x = x_accel;
  acceleration.y = y_accel;
  acceleration.z = z_accel;

  rotation_delta.time = base::Time::now();
  rotation_delta.yaw = x_delta_angle;
  rotation_delta.roll = y_delta_angle;
  rotation_delta.pitch = z_delta_angle;

  simple_orientation.time = base::Time::now();
  simple_orientation.yaw = x_angle;
  simple_orientation.roll = y_angle;
  simple_orientation.pitch = z_angle;

  std::cout << std::setprecision(5);
//  std::cout << "acc_xyz delta_xyz ownsum_xyz " << x_accel << " " << y_accel << " " << z_accel << " " << x_delta_angle << " " << y_delta_angle << " " << z_delta_angle << " " << x_angle << " " << y_angle << " " << z_angle << "\n";


//    uint16_t msg_id   = le16toh(*reinterpret_cast<uint16_t const*>(buffer));
//    switch(msg_id)
 //   {
/*    case raw::FixedLeader::ID:
        parseFixedLeader(buffer, size);
        if (cellReadings.readings.size() != acqConf.cell_count)
        {
            cellReadings.readings.resize(acqConf.cell_count);
            invalidateCellReadings();
        }
        break;
    case raw::VariableLeader::ID:
        parseVariableLeader(buffer, size);
        break;
    case raw::VelocityMessage::ID:
        cellReadings.time = status.time;
        parseVelocityReadings(buffer, size);
        break;
    case raw::CorrelationMessage::ID:
        cellReadings.time = status.time;
        parseCorrelationReadings(buffer, size);
        break;
    case raw::IntensityMessage::ID:
        cellReadings.time = status.time;
        parseIntensityReadings(buffer, size);
        break;
case raw::QualityMessage::ID:
        cellReadings.time = status.time;
        parseQualityReadings(buffer, size);
        break;
    case raw::BottomTrackingMessage::ID:
        bottomTracking.time = status.time;
        parseBottomTrackingReadings(buffer, size);
        break;*/
//    }
}

void KVH1750Parser::parseEnsemble(uint8_t const* buffer, size_t size)
{
    // Validate the message sizes
    raw::Header const& header = *reinterpret_cast<raw::Header const*>(buffer);
    

  /*  std::cout << "parseEnsemble with buffer " <<  (int)buffer[0] << " " << (int)buffer[1] << " ... " <<
         (int)buffer[size-2] << " " <<(int)buffer[size-1] << "\n";
*/

/*    if (sizeof(raw::Header) + header.msg_count * 2 > size)
        throw std::runtime_error("not enough bytes for " + lexical_cast<string>((int)header.msg_count) + " messages");

    uint32_t offsets[256];
    for (int i = 0; i < header.msg_count; ++i)
        offsets[i] = le16toh(header.offsets[i]);

    //invalidateCellReadings();
    for (int i = 0; i < header.msg_count; ++i)*/

    parseMessage(buffer + 4, size - 8);
}





fog_acceleration KVH1750Parser::getAcceleration()
{
//    std::cout << "parser cpp acc.x " << acceleration.x << std::endl;
    return acceleration;
}

fog_rotation_delta KVH1750Parser::getRotationDelta()
{
//    std::cout << "parser cpp acc.x " << acceleration.x << std::endl;
    return rotation_delta;
}

fog_simple_orientation KVH1750Parser::getSimpleOrientation()
{
//    std::cout << "parser cpp acc.x " << acceleration.x << std::endl;
    return simple_orientation;
}
