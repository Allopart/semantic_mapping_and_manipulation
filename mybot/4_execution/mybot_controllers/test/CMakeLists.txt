#include_directories(${MYBOT2_SOURCE_DIR}/src)

FIND_PACKAGE( Boost 1.50.0 COMPONENTS system regex timer thread chrono )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -O3 -ffast-math -fexpensive-optimizations -DNDEBUG")


if(Boost_FOUND)

    include_directories(${Boost_INCLUDE_DIR})
    link_directories(${Boost_LIBRARY_DIR})

    add_executable ( test_packet test_packet.cpp )
    target_link_libraries(test_packet ${Boost_LIBRARIES})

    add_executable ( test_sync test_sync.cpp )
    target_link_libraries(test_sync ${Boost_LIBRARIES})

endif()
