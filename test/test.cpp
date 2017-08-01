#define BOOST_TEST_MAIN
#define BOOST_TEST_MODULE "imu_kvh_1750"
#define BOOST_AUTO_TEST_MAIN

#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test.hpp>
#include <imu_kvh_1750/Driver.hpp>
#include <boost/crc.hpp>
#include <arpa/inet.h>

using namespace imu_kvh_1750;

class DriverTest : public Driver
{
public:
    DriverTest() : Driver() {}

    int testExtractPacket(uint8_t const* buffer, size_t buffer_size) const
    {
        return extractPacket(buffer, buffer_size);
    }
};

void createValidHeader(uint8_t* msg)
{
    for(unsigned i = 0; i < 36; ++i)
        msg[i] = (uint8_t)(rand() % 256);
    msg[0] = 0xFE;
    msg[1] = 0x81;
    msg[2] = 0xFF;
    msg[3] = 0x55;
    msg[28] = 0x77;
};

void createCRC(uint8_t* msg)
{
    boost::crc_optimal<32,0x04C11DB7,0xFFFFFFFF,0,false,false> kvh_crc;
    kvh_crc.process_bytes(msg,32);
    uint32_t* crc = reinterpret_cast<uint32_t*>(&msg[32]);
    *crc = htonl(kvh_crc.checksum());
};

BOOST_AUTO_TEST_CASE(extract_packet)
{
    int valid_msg_size = 36;
    int buffer_size = 100;
    uint8_t msg[buffer_size];

    DriverTest driver;

    // invalid start byte
    msg[0] = 0;
    BOOST_CHECK(driver.testExtractPacket(msg, 1) == -1);
    BOOST_CHECK(driver.testExtractPacket(msg, buffer_size) == -1);

    createValidHeader(msg);
    createCRC(msg);

    // valid msg
    BOOST_CHECK(driver.testExtractPacket(msg, valid_msg_size) == valid_msg_size);
    BOOST_CHECK(driver.testExtractPacket(msg, buffer_size) == valid_msg_size);

    // invalid msg size
    BOOST_CHECK(driver.testExtractPacket(msg, 0) == 0);
    BOOST_CHECK(driver.testExtractPacket(msg, valid_msg_size-1) == 0);

    // invalid header bytes
    createValidHeader(msg);
    msg[1] = 0;
    createCRC(msg);
    BOOST_CHECK(driver.testExtractPacket(msg, buffer_size) == -valid_msg_size);
    createValidHeader(msg);
    msg[2] = 0;
    createCRC(msg);
    BOOST_CHECK(driver.testExtractPacket(msg, buffer_size) == -valid_msg_size);
    createValidHeader(msg);
    msg[3] = 0;
    createCRC(msg);
    BOOST_CHECK(driver.testExtractPacket(msg, buffer_size) == -valid_msg_size);

    // invalid status byte
    createValidHeader(msg);
    msg[28] = 0;
    createCRC(msg);
    BOOST_CHECK(driver.testExtractPacket(msg, buffer_size) == -valid_msg_size);

    // invalid CRC checksum
    createValidHeader(msg);
    createCRC(msg);
    msg[32] = ~msg[32];
    BOOST_CHECK(driver.testExtractPacket(msg, buffer_size) == -valid_msg_size);
}