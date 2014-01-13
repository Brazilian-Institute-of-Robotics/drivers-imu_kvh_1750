rock_library(imu_kvh_1750
    SOURCES KVH1750Parser.cpp Driver.cpp
    HEADERS KVH1750Messages.hpp KVH1750Raw.hpp KVH1750Parser.hpp KVH1750Types.hpp Driver.hpp
    DEPS_PKGCONFIG base-types iodrivers_base)

rock_executable(imu_kvh_1750_bin Main.cpp
    DEPS imu_kvh_1750)

