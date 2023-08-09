add_executable(programing_example)
		
target_sources(programing_example PRIVATE
        main.c		
        )		

# pull in common dependencies
target_link_libraries(programing_example libfabric pico_stdlib hardware_clocks hardware_spi)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(programing_example)

# add url via pico_set_program_url
example_auto_set_url(programing_example)

pico_enable_stdio_usb(programing_example 1)
pico_enable_stdio_uart(programing_example 0)
