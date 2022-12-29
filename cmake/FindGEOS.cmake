find_path(GEOS_INCLUDE_DIRS
        NAMES
        geos
        PATHS
        /usr/include
        /usr/local/include
        )

find_library(GEOSi_CXX_LIBRARY
        NAMES
        geos
        PATHS
        /usr/lib
        /usr/local/lib
        )

find_library(GEOS_LIBRARY
        NAMES
        geos_c
        PATHS
        /usr/lib
        /usr/local/lib
        )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GEOS DEFAULT_MSG GEOS_LIBRARY)