add_library(libfabric)
		
target_sources(libfabric PRIVATE
        libfabric.c
        )		

# pull in common dependencies
target_link_libraries(libfabric pico_stdlib hardware_clocks hardware_spi)
