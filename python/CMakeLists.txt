project(python_wrapper)

pybind11_add_module(registration_python registration/python_wrapper.cpp)

target_link_libraries(registration_python PUBLIC Registration::registration)

# https://github.com/pybind/pybind11/issues/1818
if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(registration_python PUBLIC -fsized-deallocation)
endif()
