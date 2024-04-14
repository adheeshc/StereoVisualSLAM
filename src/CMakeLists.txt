add_library(mySLAM SHARED
    camera.cpp
    config.cpp
    dataset.cpp
    feature.cpp
    frame.cpp
    frontend.cpp
    map.cpp
    mapPoint.cpp
    slam.cpp
    )

target_link_libraries(mySLAM
    ${THIRD_PARTY_LIBS})