# Component Models library
# Note: Initially empty, add source files as components are implemented
add_library(icarus_components INTERFACE)

target_include_directories(icarus_components
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/icarus/components>
)

target_link_libraries(icarus_components
  INTERFACE icarus
)

add_library(icarus::components ALIAS icarus_components)
