# Helper function to create example executables with config directory
# Each example is its own subdirectory with main.cpp and config/
function(icarus_add_example NAME DIR)
  add_executable(${NAME} ${DIR}/main.cpp)
  target_link_libraries(${NAME} PRIVATE icarus)

  # Copy config directory to build output for runtime access
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${DIR}/config)
    add_custom_command(
      TARGET ${NAME} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/${DIR}/config
        $<TARGET_FILE_DIR:${NAME}>/config
    )
  endif()

  # Copy data directory if present
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${DIR}/data)
    add_custom_command(
      TARGET ${NAME} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/${DIR}/data
        $<TARGET_FILE_DIR:${NAME}>/data
    )
  endif()
endfunction()

# =============================================================================
# Intro Example (minimal, no config)
# =============================================================================
add_executable(hello_world intro/hello_world.cpp)
target_link_libraries(hello_world PRIVATE icarus)

add_executable(orbital_demo intro/orbital_demo.cpp)
target_link_libraries(orbital_demo PRIVATE icarus)
target_include_directories(orbital_demo PRIVATE ${PROJECT_SOURCE_DIR}/components)

add_executable(formation_flight_demo intro/formation_flight_demo.cpp)
target_link_libraries(formation_flight_demo PRIVATE icarus)
target_include_directories(formation_flight_demo PRIVATE ${PROJECT_SOURCE_DIR}/components)
