# CMake generated Testfile for 
# Source directory: /home/oscar/artery/scenarios
# Build directory: /home/oscar/artery/build/scenarios
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(mtits2017-test "/home/oscar/omnetpp-5.6/bin/opp_run_release" "-n" "/home/oscar/artery/src/artery:/home/oscar/artery/src/traci:/home/oscar/artery/extern/veins/examples/veins:/home/oscar/artery/extern/veins/src/veins:/home/oscar/artery/extern/inet/src:/home/oscar/artery/extern/inet/examples:/home/oscar/artery/extern/inet/tutorials:/home/oscar/artery/extern/inet/showcases" "-l" "/home/oscar/artery/build/src/artery/envmod/libartery_envmod.so" "-l" "/home/oscar/artery/build/scenarios/highway-police/libartery_police.so" "-l" "/home/oscar/artery/build/src/artery/envmod/libartery_envmod.so" "-l" "/home/oscar/artery/build/src/artery/storyboard/libartery_storyboard.so" "-l" "/home/oscar/artery/extern/inet/out/gcc-release/src/libINET.so" "-l" "/home/oscar/artery/extern/veins/out/gcc-release/src/libveins.so" "-l" "/home/oscar/artery/build/src/artery/libartery_core.so" "omnetpp.ini" "-uCmdenv" "--sim-time-limit=30s")
set_tests_properties(mtits2017-test PROPERTIES  WORKING_DIRECTORY "/home/oscar/artery/scenarios/mt-its2017")
add_test(car2car-grid-cam_bsp "/home/oscar/omnetpp-5.6/bin/opp_run_release" "-n" "/home/oscar/artery/src/artery:/home/oscar/artery/src/traci:/home/oscar/artery/extern/veins/examples/veins:/home/oscar/artery/extern/veins/src/veins:/home/oscar/artery/extern/inet/src:/home/oscar/artery/extern/inet/examples:/home/oscar/artery/extern/inet/tutorials:/home/oscar/artery/extern/inet/showcases" "-l" "/home/oscar/artery/build/src/artery/envmod/libartery_envmod.so" "-l" "/home/oscar/artery/build/scenarios/highway-police/libartery_police.so" "-l" "/home/oscar/artery/build/src/artery/envmod/libartery_envmod.so" "-l" "/home/oscar/artery/build/src/artery/storyboard/libartery_storyboard.so" "-l" "/home/oscar/artery/extern/inet/out/gcc-release/src/libINET.so" "-l" "/home/oscar/artery/extern/veins/out/gcc-release/src/libveins.so" "-l" "/home/oscar/artery/build/src/artery/libartery_core.so" "omnetpp.ini" "-uCmdenv" "-ccam_bsp" "--sim-time-limit=30s")
set_tests_properties(car2car-grid-cam_bsp PROPERTIES  WORKING_DIRECTORY "/home/oscar/artery/scenarios/car2car-grid")
subdirs(artery)
subdirs(gemv2)
subdirs(highway-police)
subdirs(rsu_grid)
subdirs(storyboard)
