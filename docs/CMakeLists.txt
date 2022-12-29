# Check if Doxygen is installed
find_package(Doxygen)

if (DOXYGEN_FOUND)
    message("Building documentation")
    # TODO: documentation coverage

    # Set input and output files
    set(doxygen_in ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
    set(doxygen_out ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    # Request to configure the files
    configure_file(${doxygen_in} ${doxygen_out})
    message("Doxygen build started")

    add_custom_target(documentation ALL
            COMMAND ${DOXYGEN_EXECUTABLE} ${doxygen_out}
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM)
else (DOXYGEN_FOUND)
    message("Doxygen needs to be installed to generate the doxygen documentation")
endif (DOXYGEN_FOUND)