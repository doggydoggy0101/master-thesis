project(registration_src)
include(GNUInstallDirs)

file(GLOB SRC_FILES src/*.cpp)
add_library(registration SHARED ${SRC_FILES})

target_include_directories(registration PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
    $<INSTALL_INTERFACE:${CMALE_INSTALL_INCLUDEDIR}> 
)

target_link_libraries(registration Eigen3::Eigen robin::robin)
add_library(Registration::registration ALIAS registration)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION include
)
install(TARGETS registration
  EXPORT RegistrationConfig
  LIBRARY DESTINATION lib
)

export(TARGETS registration
  NAMESPACE Registration::
  FILE "${CMAKE_CURRENT_BINARY_DIR}/RegistrationConfig.cmake"
)
install(EXPORT RegistrationConfig
  DESTINATION "${CMAKE_INSTALL_DATADIR}/Registration/cmake"
  NAMESPACE Registration::
)
