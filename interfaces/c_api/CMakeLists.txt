# Icarus C API
# Provides a C-compatible interface for FFI bindings

add_library(icarus_c SHARED
    icarus_c.cpp
)

target_include_directories(icarus_c
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include/icarus>
)

# Force-link icarus_components to ensure static constructors run
# This is needed for component registration to work
if(APPLE)
    target_link_libraries(icarus_c PRIVATE icarus -Wl,-force_load icarus_components)
else()
    target_link_libraries(icarus_c PRIVATE icarus -Wl,--whole-archive icarus_components -Wl,--no-whole-archive)
endif()

# Pass version info to the implementation
target_compile_definitions(icarus_c PRIVATE
    ICARUS_VERSION_STRING="${PROJECT_VERSION}"
    ICARUS_VERSION_MAJOR=${PROJECT_VERSION_MAJOR}
    ICARUS_VERSION_MINOR=${PROJECT_VERSION_MINOR}
    ICARUS_VERSION_PATCH=${PROJECT_VERSION_PATCH}
    ICARUS_C_BUILDING_DLL
)

# Symbol visibility
set_target_properties(icarus_c PROPERTIES
    C_VISIBILITY_PRESET hidden
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)

# Public header
set_target_properties(icarus_c PROPERTIES
    PUBLIC_HEADER icarus.h
)

# Install targets
install(TARGETS icarus_c
    EXPORT icarusTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/icarus
)

# pkg-config file for Unix systems
if(UNIX)
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/icarus.pc.in
        ${CMAKE_CURRENT_BINARY_DIR}/icarus.pc
        @ONLY
    )
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/icarus.pc
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig
    )
endif()
