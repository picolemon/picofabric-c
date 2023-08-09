#pragma once

#include <stdint.h>

//#define FABRIC_DEBUG 1


/** Supported board Ids
*/
enum FPGABoardId {
	BOARD_ANY = 0x0,
    BOARD_FABRIC12k = 0x1
};


/* Device ID's reported by FPGA chip.
*/
enum FPGADeviceIds
{
    FPGA_DEVID_LFE5U_12 = 0x01111043,
    FPGA_DEVID_LFE5U_25 = 0x01111043,
    FPGA_DEVID_LFE5U_45 = 0x01112043,
    FPGA_DEVID_LFE5U_85 = 0x01113043
};


/** ECP5 FPGA SPI commands
*/
enum FPGACommands
{
    FPGA_CMD_LSC_READ_STATUS    = 0x3C,
    FPGA_CMD_READ_ID            = 0xE0,
    FPGA_CMD_USERCODE           = 0xC0,
    FPGA_CMD_ISC_ENABLE         = 0xC6,
    FPGA_CMD_LSC_BITSTREAM_BURST= 0x7A,
    FPGA_CMD_LSC_CHECK_BUSY     = 0xF0,
  	FPGA_CMD_ISC_DISABLE        = 0x26
};


/** FPGA SPI interface config.
*/
typedef struct FPGA_config_t
{
    int csn;
    int sck;
    int mosi;
    int miso;
    int programn;
    int spiId;
    int is_initialized;
    enum FPGABoardId board_id;
} FPGA_config;


/** Default FPGA HW pin mapping
*/
#define FPGA_DEFAULT_CSN 13
#define FPGA_DEFAULT_SCK 10
#define FPGA_DEFAULT_MOSI 11
#define FPGA_DEFAULT_MISO 12
#define FPGA_DEFAULT_PROGRAMN 15
#define FPGA_DEFAULT_SPIID 1


/** Initialise the FPGA default configuration object.
@param FPGA_config_t config 	Configuration struct to initialize.
@returns int    Configuration success.
*/
int fpga_init_config( struct FPGA_config_t* config, enum FPGABoardId board_id );


/** Read data from the FPGA over spi.
@param FPGA_config_t config 	Configuration object.
@param uint8_t config   Command to execute. see FPGACommands.
@param uint8_t* buf   Buffer to read.
@param uint32_t len   Size of buffer to read.
*/
void fpga_read_spi( struct FPGA_config_t* config, uint8_t cmd, uint8_t *buf, uint32_t len);


/** Read FPGA device ID. 
@param FPGA_config_t config 	Configuration object.
@returns uint32_t    Returns read device id eg. FPGA_DEVID_LFE5U_12
*/
uint32_t fpga_read_id( struct FPGA_config_t* config );


/** Read FPGA status register.
@param FPGA_config_t config 	Configuration object.
@returns uint32_t    Returns status bits.
*/
uint32_t fpga_read_status( struct FPGA_config_t* config );


/** FPGA enables ISC mode.
@param FPGA_config_t config 	Configuration object.
*/
void fpga_isc_enable( struct FPGA_config_t* config );


/** FPGA disabled ISC mode.
@param FPGA_config_t config 	Configuration object.
*/
void fpga_isc_disable( struct FPGA_config_t* config );


/** Read FPGA busy status register.
@param FPGA_config_t config 	Configuration object.
@returns uint8_t    Returns busy bits.
*/
uint8_t fpga_poll_busy( struct FPGA_config_t* config );


/** Being writing bitstream, device must be in ISC mode before calling. 
* Used in combination with fpga_write_bitstream_block & fpga_write_bitstream_end.
@param FPGA_config_t config 	Configuration object.
*/
void fpga_write_bitstream_begin( struct FPGA_config_t* config );


/** Write bitstream block to fpga, this can be called when data is read and fed to the FPGA, 
* this expects the device to be in ISC mode and fpga_write_bitstream_begin called before.
@param FPGA_config_t config 	Configuration object.
@param uint8_t* buf   Buffer block to write.
@param uint32_t len   Size of buffer block to write.
*/
void fpga_write_bitstream_block( struct FPGA_config_t* config, uint8_t* buf, uint32_t len );


/** End writing bitstream.
@param FPGA_config_t config 	Configuration object.
*/
void fpga_write_bitstream_end( struct FPGA_config_t* config );


/** Write bitstream to fpga this expects the device to be in ISC mode.
@param FPGA_config_t config 	Configuration object.
@param uint8_t* buf   bitstream bufferto  write.
@param uint32_t len   Size of bitstream buffer to write.
*/
void fpga_write_bitstream( struct FPGA_config_t* config, uint8_t* buf, uint32_t len );


/** Program the FPGA with a given bitstream buffer.
@param FPGA_config_t config 	Configuration object.
@param uint8_t* buf   bitstream bufferto  write.
@param uint32_t len   Size of bitstream buffer to write.
*/
int fpga_program_device( struct FPGA_config_t* config, uint8_t* buf, uint32_t len );


/** [internal] Init debug log port
*/
int libfabric_debug_init( int uartId, int txPin );


/** [internal] Debug log message.
*/
void libfabric_debug_puts( const char * buf );