# testing binary
add_executable(cpp_test test_test.cpp)
add_test(
    NAME test_test
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMAND test_test
)