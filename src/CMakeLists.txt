add_library(mySLAM SHARED
    frame.cpp
    feature.cpp
    mapPoint.cpp
    map.cpp
    camera.cpp
    config.cpp
    slam.cpp)

target_link_libraries(mySLAM
    ${THIRD_PARTY_LIBS})