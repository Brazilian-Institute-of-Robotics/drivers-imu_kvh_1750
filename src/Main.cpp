#include <iostream>
#include <imu_kvh_1750/Driver.hpp>

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
    }
	return 0;
}
