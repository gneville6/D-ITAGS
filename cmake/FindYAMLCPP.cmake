# attempt to find static library first if this is set
if (YAMLCPP_STATIC_LIBRARY)
    set(YAMLCPP_STATIC libyaml-cpp.a)
endif ()

# find the yaml-cpp include directory
find_path(YAMLCPP_INCLUDE_DIR yaml-cpp/yaml.h
        PATHS
        /usr/include/
        /usr/local/include/)

# find the yaml-cpp library
find_library(YAMLCPP_LIBRARY
        NAMES ${YAMLCPP_STATIC} yaml-cpp
        PATHS
        /usr/lib
        /usr/local/lib)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(YAMLCPP DEFAULT_MSG YAMLCPP_INCLUDE_DIR YAMLCPP_LIBRARY)
mark_as_advanced(YAMLCPP_INCLUDE_DIR YAMLCPP_LIBRARY)