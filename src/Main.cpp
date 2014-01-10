#include <iostream>
#include <fog_kvh_1750/Driver.hpp>

using namespace fog_kvh_1750;

void usage()
{
    std::cerr << "fog_kvh_1750_bin DEVICE" << std::endl;
}


int main(int argc, char const* argv[])
{
//	fog_kvh_1750::DummyClass dummyClass;
//	dummyClass.welcome();

       if (argc != 2)
    {
        usage();
        return 1;
    }

    fog_kvh_1750::Driver driver;
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

        //BottomTracking const& tracking = driver.bottomTracking;
        //std::cout << tracking.time.toString() << " " << driver.status.seq;
    }


	return 0;
}
