file (GLOB SOURCES "*.cpp")
add_executable (GeneratorOfDatasets ${SOURCES})
install (TARGETS GeneratorOfDatasets DESTINATION bin)
