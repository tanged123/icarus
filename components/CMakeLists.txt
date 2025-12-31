# Component Models library
# Contains built-in components and factory registration

add_library(icarus_components
  registration.cpp
)

target_include_directories(icarus_components
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/icarus/components>
)

target_link_libraries(icarus_components
  PUBLIC icarus
)

add_library(icarus::components ALIAS icarus_components)
