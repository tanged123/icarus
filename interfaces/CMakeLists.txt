# External Interfaces (C API, Python, MATLAB)
# Phase 7 implementation

# C API (always built when BUILD_INTERFACES is ON)
add_subdirectory(c_api)

# Python bindings (pybind11) - Phase 7.2
if(BUILD_PYTHON)
    find_package(pybind11 QUIET)
    if(pybind11_FOUND)
        add_subdirectory(python)
    else()
        message(WARNING "pybind11 not found - Python bindings disabled")
    endif()
endif()

# MATLAB bindings - Phase 7.3 (optional)
# add_subdirectory(matlab)
