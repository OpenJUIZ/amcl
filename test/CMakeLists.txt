add_executable(localization_test localization_test.cpp)
target_link_libraries(localization_test PRIVATE amcl::amcl)
add_test(NAME localization_test COMMAND localization_test)
