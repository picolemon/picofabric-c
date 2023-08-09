#include <stdio.h>
#include "../libfabric/libfabric.h"
#include "bitstream.bit.h"

int main()
{
    struct FPGA_config_t config;	
    fpga_init_config( &config, BOARD_FABRIC12k );	
    fpga_program_device( &config, bitstream, bitstream_size );		
	return 0;
}