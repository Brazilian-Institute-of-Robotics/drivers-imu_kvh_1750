#include <iostream>
#include <imu_kvh_1750/Driver.hpp>
#include <base/samples/IMUSensors.hpp>
#include <math.h>
#include <unistd.h>
//#include <imu_kvh_1750/KVH1750Types.hpp>

using namespace imu_kvh_1750;

void usage()
{
    std::cerr << "fog_kvh_1750_bin DEVICE" << std::endl;
}

inline double to_degrees(double radians){
  return radians*(180.0/M_PI);
}

int main(int argc, char const* argv[])
{
    double yaw = 0;

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
        yaw += imu.gyro[2];
//        std::cout << "Acceleration: " << imu.acc[0]  << "|" << imu.acc[1] << "|" << imu.acc[2] << std::endl;
//        std::cout << "Rotation: " << imu.gyro[0]  << "|" << imu.gyro[1] << "|" << imu.gyro[2] << std::endl;
        std::cout << "Accumulated Yaw (degrees): " << to_degrees(yaw) << " Counter: " << driver.getCounter() << " Temperature: " << driver.getTemperature() << std::endl;
        usleep(1);
    }
	return 0;
}
