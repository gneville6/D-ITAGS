find_path(GUROBI_INCLUDE_DIRS
        NAMES
        gurobi_c.h
        gurobi_c++.h
        HINTS
        ${GUROBI_DIR}
        $ENV{GUROBI_HOME}
        /opt/gurobi/linux64
        /opt/gurobi911/linux64
        /opt/gurobi912/linux64
        /opt/gurobi950/linux64
        PATH_SUFFIXES
        include)

find_library(GUROBI_LIBRARY
        NAMES
        gurobi
        gurobi91
        gurobi95
        HINTS
        ${GUROBI_DIR}
        $ENV{GUROBI_HOME}
        /opt/gurobi/linux64
        /opt/gurobi911/linux64
        /opt/gurobi912/linux64
        /opt/gurobi950/linux64
        PATH_SUFFIXES
        lib)

if ("CXX" IN_LIST LANGUAGES)
    find_library(GUROBI_CXX_LIBRARY
            NAMES
            gurobi_c++
            HINTS
            ${GUROBI_DIR}
            $ENV{GUROBI_HOME}
            /opt/gurobi/linux64
            /opt/gurobi911/linux64
            /opt/gurobi912/linux64
            /opt/gurobi950/linux64
            PATH_SUFFIXES
            lib)
endif ()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GUROBI DEFAULT_MSG GUROBI_LIBRARY)