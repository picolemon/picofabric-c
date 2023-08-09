add_executable(fabric_bootloader)
		
target_sources(fabric_bootloader PRIVATE
        fabric_bootloader.c
		miniz.c		
        )		

# pull in common dependencies
target_link_libraries(fabric_bootloader libfabric pico_stdlib pico_unique_id_headers hardware_clocks hardware_spi)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(fabric_bootloader)

# add url via pico_set_program_url
example_auto_set_url(fabric_bootloader)

pico_enable_stdio_usb(fabric_bootloader 1)
pico_enable_stdio_uart(fabric_bootloader 0)
