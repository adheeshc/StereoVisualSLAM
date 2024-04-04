add_library(mySLAM SHARED
    frame.cpp
    feature.cpp
    mapPoint.cpp)

target_link_libraries(mySLAM
    ${THIRD_PARTY_LIBS})