rock_library(imu_kvh_1750
    SOURCES Driver.cpp
    HEADERS Driver.hpp
    DEPS_PKGCONFIG base-types iodrivers_base)

rock_executable(imu_kvh_1750_bin Main.cpp
    DEPS imu_kvh_1750)

#rock_find_cmake(Boost REQUIRED)
