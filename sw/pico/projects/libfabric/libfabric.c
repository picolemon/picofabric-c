#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "libfabric.h"

/** Debug uart
*/
#ifdef FABRIC_DEBUG	
	// Debug globals
	uart_inst_t* _debug_uart = 0;
	char _debugtmp[128];
	#define DEBUG_PRINT(fmt, args...)    {snprintf(_debugtmp, sizeof(_debugtmp), fmt, ## args); libfabric_debug_puts(_debugtmp);}
#else
	#define DEBUG_PRINT(fmt, args...)
#endif


static spi_inst_t * select_spi(int spiId) 
{
	switch(spiId)	
	{
		case 0:
			return spi0;
		case 1:
			return spi1;
	}
	return 0;
}


int fpga_init_config( struct FPGA_config_t* config, enum FPGABoardId board_id )
{
	if(!config)
		return 0;

	// Init with default SPI config
    config->csn = FPGA_DEFAULT_CSN;
    config->sck = FPGA_DEFAULT_SCK;
    config->mosi = FPGA_DEFAULT_MOSI;
    config->miso = FPGA_DEFAULT_MISO;
    config->programn = FPGA_DEFAULT_PROGRAMN;
    config->spiId = FPGA_DEFAULT_SPIID;
	config->is_initialized = 1;
	config->board_id = board_id;

	// programn pin setup
	gpio_init(config->programn);	
    gpio_set_dir(config->programn, GPIO_OUT);
	gpio_put(config->programn, 1);
	sleep_ms(100);
	
    // Spi configured at 1 MHz
    spi_init(select_spi(config->spiId), 1000000);
    gpio_set_function(config->miso, GPIO_FUNC_SPI);
    gpio_set_function(config->sck, GPIO_FUNC_SPI);
    gpio_set_function(config->mosi, GPIO_FUNC_SPI);

    gpio_init(config->csn);
    gpio_put(config->csn, 1);
    gpio_set_dir(config->csn, GPIO_OUT);   
    
	sleep_ms(50);

	return 1;
}


void fpga_read_spi( struct FPGA_config_t* config, uint8_t cmd, uint8_t *buf, uint32_t len) {
    uint8_t dataout[] = {
		cmd
    };
    gpio_put(config->csn, 0);
    spi_write_blocking(select_spi(config->spiId), dataout, 1);
    spi_read_blocking(select_spi(config->spiId), 0, buf, len);
    gpio_put(config->csn, 1);
}


uint32_t fpga_read_id( struct FPGA_config_t* config )
{	  
	uint8_t buf[8];
	int readSz = 4;		
	fpga_read_spi( config, FPGA_CMD_READ_ID, buf, 3 + readSz );
			
	return (buf[3] << 24) | (buf[4] << 16) | (buf[5] << 8) | (buf[6] << 0);
}


uint32_t fpga_read_status( struct FPGA_config_t* config )
{	  
	uint8_t buf[32];
	int readSz = 4;		
	fpga_read_spi( config, FPGA_CMD_LSC_READ_STATUS, buf, 3+readSz);
			
	return (buf[3] << 24) | (buf[4] << 16) | (buf[5] << 8) | (buf[6] << 0);
}


void fpga_isc_enable( struct FPGA_config_t* config )
{	  
	uint8_t buf[32];	
	fpga_read_spi( config, FPGA_CMD_ISC_ENABLE, buf, 3); // cmd + dummy ( Class C cmd ) 
}


void fpga_isc_disable( struct FPGA_config_t* config )
{	  
	uint8_t buf[32];	
	fpga_read_spi( config, FPGA_CMD_ISC_DISABLE, buf, 3); // cmd + dummy ( Class C cmd ) 
}


uint8_t fpga_poll_busy( struct FPGA_config_t* config )
{	  
	uint8_t buf[32];
	int readSz = 1;		
	fpga_read_spi( config, FPGA_CMD_LSC_CHECK_BUSY, buf, 3+readSz);
	
	return buf[3];
}


void fpga_write_bitstream_begin( struct FPGA_config_t* config )
{	 
	uint8_t burstCmd[] = { FPGA_CMD_LSC_BITSTREAM_BURST, 0, 0, 0 };		
	gpio_put(config->csn, 0);
	spi_write_blocking(select_spi(config->spiId), burstCmd, 4);
}


void fpga_write_bitstream_block( struct FPGA_config_t* config, uint8_t* data, uint32_t size )
{	 
	spi_write_blocking(select_spi(config->spiId), data, size);	
}


void fpga_write_bitstream_end( struct FPGA_config_t* config )
{	 	
	gpio_put(config->csn, 1);
	sleep_ms(100);
}


void fpga_write_bitstream( struct FPGA_config_t* config, uint8_t* buf, uint32_t len )
{	 
	// Burst write bitstream
	uint8_t burstCmd[] = { FPGA_CMD_LSC_BITSTREAM_BURST, 0, 0, 0 };		
	gpio_put(config->csn, 0);
	spi_write_blocking(select_spi(config->spiId), burstCmd, 4); // Write cmd
	spi_write_blocking(select_spi(config->spiId), buf, len); // Write bitstream payload	in burst
	gpio_put(config->csn, 1);

	sleep_ms(100);
}


int fpga_program_device( struct FPGA_config_t* config, uint8_t* buf, uint32_t len )
{
	uint8_t isBusy;

	// Enter programming mode
	DEBUG_PRINT("Toggle FPGA_PROGRAMN_PIN (Enter init mode)\r\n");
	gpio_put(config->programn, 0);
	sleep_ms(100);		
	gpio_put(config->programn, 1);
	sleep_ms(100);

	// Validate device id
	uint32_t deviceId = fpga_read_id( config );
	DEBUG_PRINT("Read DeviceId: %X\n", deviceId); 		

	int isValid = 0;
	switch(deviceId & 0x0FFFFFFF)
	{
		//case FPGA_DEVID_LFE5U_12:			
		case FPGA_DEVID_LFE5U_25:					
		//case FPGA_DEVID_LFE5U_45:					
		case FPGA_DEVID_LFE5U_85:					
			isValid = 1;
			break;							
	};	
	
	if(!isValid)
	{	
		DEBUG_PRINT("Failed, invalid device Id: %X\n", deviceId); 
		return 0;
	}
	
	// Ensure device not busy
	isBusy = fpga_poll_busy( config );	

	if(isBusy)
	{
		DEBUG_PRINT("Device busy\r\r\n"); 
		return 0;
	}

	// Enter ISC mode	
	fpga_isc_enable( config );
	
	// Write bitstream
	fpga_write_bitstream( config, buf, len );
	
	// Disable ISB mode
	fpga_isc_disable( config );

	// Check complete
	isBusy = fpga_poll_busy( config );	
	if(isBusy)
	{
		DEBUG_PRINT("Device busy\r\n"); 
		return 0;
	}
	
	return 1;
}


int libfabric_debug_init( int uartId, int txPin )
{
#if FABRIC_DEBUG	
	uart_inst_t* uart = uartId == 0 ? uart0 : uart1;
    uart_init(uart, 2400);    
    gpio_set_function(txPin, GPIO_FUNC_UART);    
    int __unused actual = uart_set_baudrate(uart, 115200);
    uart_set_hw_flow(uart, false, false);
    uart_set_format(uart, 8, 1, UART_PARITY_NONE);
	
	_debug_uart = uart;
	return 1;
#else
	return 0;
#endif
}


void libfabric_debug_puts( const char * buff )
{
#if FABRIC_DEBUG		
	if(!_debug_uart)
		return;
	uart_puts( _debug_uart, buff);
#endif	
}