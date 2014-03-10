#include <iostream>
#include <imu_kvh_1750/Driver.hpp>
#include <base/samples/IMUSensors.hpp>
//#include <imu_kvh_1750/KVH1750Types.hpp>

using namespace imu_kvh_1750;

void usage()
{
    std::cerr << "fog_kvh_1750_bin DEVICE" << std::endl;
}


int main(int argc, char const* argv[])
{

       if (argc != 2)
    {
        usage();
        return 1;
    }

    imu_kvh_1750::Driver driver;
    std::cout << "opening ...";
    driver.open(argv[1]);
    std::cout << "setting timeout...";
    driver.setReadTimeout(base::Time::fromSeconds(5));

    std::cout << "reading ...";
    driver.read();
    std::cout << "done\n";

    while(true)
    {
        driver.read();
        base::samples::IMUSensors imu = driver.getIMUReading();
        std::cout << "Acceleration: " << imu.acc[0]  << "|" << imu.acc[1] << "|" << imu.acc[2] << std::endl;
        std::cout << "Rotation: " << imu.gyro[0]  << "|" << imu.gyro[1] << "|" << imu.gyro[2] << std::endl;
    }
	return 0;
}
