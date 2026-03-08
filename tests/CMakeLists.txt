add_executable(unit_tests
    test_main.cpp
    test_curvature.cpp
    test_path_length.cpp
    test_cleaning_area.cpp
    test_json.cpp
)

target_link_libraries(unit_tests PRIVATE Catch2::Catch2WithMain robot_cleaner_core)

# Register tests to CTest
include(CTest)
include(Catch)
catch_discover_tests(unit_tests)
