# Python Bindings (PyBind11)
# Phase 7.2 implementation

find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

# Allow overriding Python install path (for Nix builds)
if(NOT ICARUS_PYTHON_INSTALL_DIR)
    set(ICARUS_PYTHON_INSTALL_DIR "${Python_SITEARCH}")
endif()

# Build the extension module
pybind11_add_module(_icarus MODULE
    icarus_python.cpp
)

# Force-link icarus_components to ensure static constructors run
# This is needed for component registration to work
if(APPLE)
    target_link_libraries(_icarus PRIVATE icarus -Wl,-force_load icarus_components)
else()
    target_link_libraries(_icarus PRIVATE icarus -Wl,--whole-archive icarus_components -Wl,--no-whole-archive)
endif()
target_compile_definitions(_icarus PRIVATE
    ICARUS_VERSION_STRING="${PROJECT_VERSION}"
    ICARUS_VERSION_MAJOR=${PROJECT_VERSION_MAJOR}
    ICARUS_VERSION_MINOR=${PROJECT_VERSION_MINOR}
    ICARUS_VERSION_PATCH=${PROJECT_VERSION_PATCH}
)

# Copy module to build directory for testing
set(PYTHON_OUTPUT_DIR "${CMAKE_BINARY_DIR}/python/icarus")
add_custom_command(TARGET _icarus POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${PYTHON_OUTPUT_DIR}"
    COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:_icarus>" "${PYTHON_OUTPUT_DIR}/"
    COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/icarus/__init__.py" "${PYTHON_OUTPUT_DIR}/"
    COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/icarus/py.typed" "${PYTHON_OUTPUT_DIR}/"
    COMMENT "Copying Python module to build directory"
)

# Install to Python site-packages (for pip install)
install(TARGETS _icarus
    LIBRARY DESTINATION "${ICARUS_PYTHON_INSTALL_DIR}/icarus"
)

install(FILES
    icarus/__init__.py
    icarus/py.typed
    DESTINATION "${ICARUS_PYTHON_INSTALL_DIR}/icarus"
)

message(STATUS "Python bindings enabled (Python ${Python_VERSION})")
