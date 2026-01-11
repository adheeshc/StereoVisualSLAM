add_library(mySLAM SHARED
    backend.cpp
    camera.cpp
    config.cpp
    dataset.cpp
    feature.cpp
    frame.cpp
    frontend.cpp
    map.cpp
    mapPoint.cpp
    slam.cpp
    viewer.cpp
    )

target_link_libraries(mySLAM
    ${THIRD_PARTY_LIBS})