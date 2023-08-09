/**
FPGA bitstream programmer, reads the bitstream from the PC host  and writes it to the FPGA over SPI.
Provides a simple packet based binary interface with compression.

A Debug port can be enabled by setting ENABLE_DEBUG_LOG preprocessor as the USB uart is used for comms.
*/
#include "fabric_bootloader.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/unique_id.h"
#include "hardware/clocks.h"
#include "hardware/flash.h"
#include "hardware/uart.h"
#include "hardware/sync.h"
#include "miniz.h"


/** Debug uart
*/
#define DEBUG_UART_ID uart0
#define DEBUG_UART_TX_PIN 0
#define DEBUG_UART_RX_PIN 1
const uint LED_PIN = PICO_DEFAULT_LED_PIN;

#ifdef ENABLE_DEBUG_LOG
#define DEBUG_PRINT(fmt, args...)    {sprintf(tmp, fmt, ## args); debugLog(tmp);}
#else
#define DEBUG_PRINT(fmt, args...)
#endif
	
	
/** Flash page setup 1MB storage( 4096 * 256 )
*/
#define FLASH_MAX_SECTOR 256
#define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - (FLASH_SECTOR_SIZE * FLASH_MAX_SECTOR ))
#define FLASH_MAGIC_0 0xf1f0de0e
#define FLASH_MAX_BLOCK_CNT 2048
#define FLASH_BLOCK_TO_SECTOR(blockId) (FLASH_TARGET_OFFSET + ((blockId+1) * FLASH_SECTOR_SIZE) + (0 * FLASH_PAGE_SIZE))


// Globals
static char tmp[64];
uint8_t requestPacket[4090];
uint8_t uncompressedData[4090];
	
static void debugLog(const char* msg)
{
	int sz = strlen(msg);
	for(int i=0;i<sz;i++)
		uart_putc(DEBUG_UART_ID, msg[i]);
}

void print_buf(const uint8_t *buf, size_t len) 
{
    for (size_t i = 0; i < len; ++i) 
	{
        DEBUG_PRINT("%02x", buf[i]);								
		debugLog(tmp);						
        if (i % 16 == 15)
            debugLog("\r\n");
        else
            debugLog(" ");
    }
}

/** CRC8 block of data.
*/
uint8_t crc8_block(  uint8_t* data, uint16_t sz )
{
	int crc = 0;
	for(int i=0;i<sz;i++)
		crc += data[i];
	crc = crc & 0xff;
	
	return crc;
}


/** Write single byte to uart
*/
void write_byte( uint8_t value )
{
	fwrite(&value, 1, 1, stdout );
	fflush(stdout);
}


/* Write block to uart
*/
void writeBlock( uint8_t* data, uint16_t sz)
{
#ifdef DEBUG_LOW_LEVEL_PROTOCOL
	DEBUG_PRINT("writeBlock[%X]: %d\r\n", data, sz);
#endif
		
	uint16_t packet_sz = sz + 1;
	int crc = 0;

	write_byte(FPacketHeaderMagic);	

	write_byte((packet_sz >> 0) & 0xff);	

	write_byte((packet_sz >> 8) & 0xff);		

	for (int i = 0; i < sz; i++) {			

		crc = crc + data[i];
					
#ifdef DEBUG_LOW_LEVEL_PROTOCOL		
		DEBUG_PRINT("WriteByte[%d]: %d\r\n", i, data[i]);
#endif
		
		write_byte(data[i]);	
		
		tight_loop_contents();
	}
				
    crc = crc & 0xff;

#ifdef DEBUG_LOW_LEVEL_PROTOCOL
	DEBUG_PRINT("Write packet crc: %d\r\n", crc );
#endif
	
    write_byte( crc );
}


/** Read block from uart.
*/
int readBlock( uint8_t* data, int maxSz )
{
	int crc = 0;
	int header = 0;
	uint16_t sz = 0;
	int ch = 0;
	int debug = 0;
	
	// wait for header, fast poll
	if ((header = getchar_timeout_us(0)) == PICO_ERROR_TIMEOUT) 
	{						
		return 0;
	} 
	else 
	{			
		// validate header
		if( header != FPacketHeaderMagic)
			return 0;

		// wait for size0
		sz = 0;
		if ((ch = getchar_timeout_us(100000)) == PICO_ERROR_TIMEOUT) 
		{						
			return 0;
		} 
		else 
		{	
			sz = ch & 0xff;
			
			// wait for size1
			if ((ch = getchar_timeout_us(100000)) == PICO_ERROR_TIMEOUT) 
			{						
				return 0;
			} 
			else 
			{	
				sz |= (ch & 0xff) << 8;
		
				if(debug)
				{
					DEBUG_PRINT("Begin: %d[%d]\r\n", sz-1, sz);
				}
					
				if( sz > maxSz )
				{
					DEBUG_PRINT("Max packet size: %d, max: %d\r\n", sz, maxSz );
					return 0;
				}
				
				// read off packet
				bool isTimeout = false;
				crc = 0;
				for (int i = 0; i < sz; i++) {
					if ((ch = getchar_timeout_us(100000)) == PICO_ERROR_TIMEOUT)   // 100ms timeout during block xfer
					{
					
						if(debug)
							DEBUG_PRINT("timeout\r\n");		
						
						isTimeout = true;
						break;
					}
					
					if(debug)
						DEBUG_PRINT("ReadByte[%d]: %d\r\n", i, ch);						
					
					if( i < sz-1 )
						crc += ch;
						
					data[i] = ch;
				}
				
				// Check timeout while reading
				if( isTimeout )				
					return 0;
				
				// Verify crc ( last byte )
				crc = crc & 0xff;
				int expect_crc = data[sz-1] & 0xff;
				
				if(debug)
				{
					DEBUG_PRINT("Packet: %d, crc: %d, expect_crc: %d\r\n", sz-1, crc, expect_crc );					
				}
				
				// Crc fail
				if(expect_crc != crc)
				{
					if(debug)
					{
						DEBUG_PRINT("crc fail\r\n" );						
					}
					return 0;
				}
				
				return sz - 1;
			}
		}
	}
}


/** find_bitstream_info_flash
*/
struct FBitstreamFlashInfo* find_bitstream_info_flash(  )
{
	int pageId = 0;
	uint8_t * addr = XIP_BASE + FLASH_TARGET_OFFSET + (pageId * FLASH_PAGE_SIZE);
	struct FBitstreamFlashInfo* info = (struct FBitstreamFlashInfo*)addr;
	if(info->magic0 == FLASH_MAGIC_0)
	{
		return info;
	}
	return 0;
}


/** Write bitstream info
*/
int write_bitstream_info_flash( struct FBitstreamFlashInfo* info )
{
	int pageId = 0;
	
	// See if data is the same to avoid writing
	uint8_t * addr = XIP_BASE + FLASH_TARGET_OFFSET + (pageId * FLASH_PAGE_SIZE);
	struct FBitstreamFlashInfo* info_on_flash = (struct FBitstreamFlashInfo*)addr;
	if( memcmp(info_on_flash, info,  sizeof(struct FBitstreamFlashInfo) ) == 0)
	{		
		return 1; // Same
	}
	
	// erase sector to 0xFF
	uint32_t ints = save_and_disable_interrupts();
	flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);		
	restore_interrupts (ints);

	// write info to page0 in sector ( no wear leveling )
	ints = save_and_disable_interrupts();
	uint8_t buff[FLASH_PAGE_SIZE];
	memset(buff, 0xff, FLASH_PAGE_SIZE); // Set to 0xff
	memcpy(buff, info, sizeof(struct FBitstreamFlashInfo) );
	flash_range_program(FLASH_TARGET_OFFSET + (pageId*FLASH_PAGE_SIZE), (uint8_t *)buff, FLASH_PAGE_SIZE);
	restore_interrupts (ints);
	
	// Readback from flash, ensure not worn down ( no wear leveling is done so more than possible )
	addr = XIP_BASE + FLASH_TARGET_OFFSET + (pageId * FLASH_PAGE_SIZE);
	info_on_flash = (struct FBitstreamFlashInfo*)addr;
	if( memcmp(info_on_flash, info,  sizeof(struct FBitstreamFlashInfo) ) != 0)
	{		
		return 0;	
	}
	
	return 1;
}


/** write bitstream block, allocated 1 per sector for simplicity. VERY wasteful and assumes this
is a dedicated storage chip for bitstreams.
*/
int write_bitstream_block_flash( int blockId, uint8_t* data, uint32_t size, int* crc )
{
	if(size + sizeof(struct FBitstreamBlockInfo) > FLASH_SECTOR_SIZE)
	{
		DEBUG_PRINT("size %d > FLASH_SECTOR_SIZE %d\r\n", size, FLASH_SECTOR_SIZE);
		return 0;
	}

	// add to crc
	int blockCrc = 0;
	for(int i=0;i<size;i++)
	{
		blockCrc += data[i]; // Local block cfc
		*crc = *crc + data[i]; // Complete bitstream crc ( written at end )
	}
	
	blockCrc = blockCrc & 0xff;
	
	// erase sector to 0xFF
	uint32_t ints = save_and_disable_interrupts();
	flash_range_erase(FLASH_BLOCK_TO_SECTOR(blockId), FLASH_SECTOR_SIZE);		
	restore_interrupts (ints);

	// write info to page0 in sector ( no wear leveling )
	ints = save_and_disable_interrupts();
	uint8_t buff[FLASH_SECTOR_SIZE];
	memset( buff, 0xff, FLASH_SECTOR_SIZE); // Set to 0xff
	
	struct FBitstreamBlockInfo blockInfo;
	blockInfo.blockId = blockId;
	blockInfo.blockSz = size;
	blockInfo.blockCrc = blockCrc;	
	memcpy( buff, &blockInfo, sizeof(struct FBitstreamBlockInfo) );	
	
	memcpy( buff + sizeof(struct FBitstreamBlockInfo), data, size );
	
	flash_range_program(FLASH_BLOCK_TO_SECTOR(blockId), (uint8_t *)buff, FLASH_SECTOR_SIZE);
	restore_interrupts (ints);
		
	// Readback info
	uint8_t * addr = XIP_BASE + FLASH_BLOCK_TO_SECTOR(blockId);
	struct FBitstreamBlockInfo* block_info_on_flash = (struct FBitstreamBlockInfo*)addr;
	if( memcmp( block_info_on_flash, &blockInfo, sizeof(struct FBitstreamBlockInfo) ) != 0 )
	{		
		DEBUG_PRINT("memcmp FBitstreamBlockInfo failed to block %d\r\n", blockId); 
		return 0;	
	}
	
	// Readback from flash, ensure not worn down ( no wear leveling is done so more than possible )	
	if( memcmp( addr + sizeof(struct FBitstreamBlockInfo), data, size ) != 0 )
	{		
		DEBUG_PRINT("memcmp bit data failed to block %d, size: %d\r\n", blockId, size );
		return 0;	
	}
	
	return 1;
}


/** Verify bitstream in flash storage.
*/
int verify_bitstream_flash( struct FBitstreamFlashInfo* info )
{
	if(!info)
		return 0;
	
	// Validate header
	if(info->blockCnt > FLASH_MAX_BLOCK_CNT)
	{		
		DEBUG_PRINT("info->blockCnt %d > FLASH_MAX_BLOCK_CNT %d\r\n", info->blockCnt, FLASH_MAX_BLOCK_CNT);		
		return 0;
	}
	if(info->crc + 1 != info->bitStreamCrc1)
	{
		DEBUG_PRINT("info->crc %d + 1 != info->bitStreamCrc1 %d\r\n", info->crc, info->bitStreamCrc1); 	
		return 0;
	}
	if(info->crc + 2 != info->bitStreamCrc2)
	{
		DEBUG_PRINT("info->crc %d + 2 != info->bitStreamCrc2 %d\r\n", info->crc, info->bitStreamCrc2); 		
		return 0;
	}	
	
	// Validate each block
	int crc = 0;
	for(int i=0;i<info->blockCnt;i++)
	{
		int pageId = 0; // Always use page0 in sector, no wear leveling
		int blockId = i;
		uint8_t * addr = XIP_BASE + FLASH_BLOCK_TO_SECTOR(blockId);
		
		struct FBitstreamBlockInfo* blockInfo = (struct FBitstreamBlockInfo*)addr;
		if(blockInfo->blockSz > FLASH_SECTOR_SIZE)
		{
			DEBUG_PRINT( "blockInfo[%d]->blockSz %d > FLASH_SECTOR_SIZE %d\r\n", blockId, blockInfo->blockSz , FLASH_SECTOR_SIZE); 
			return 0;
		}
		if(blockInfo->blockId != blockId)
		{
			DEBUG_PRINT( "blockInfo[%d]->blockId %d != blockId %d\r\n", blockId, blockInfo->blockId, blockId); 
			return 0;
		}
		
		// Offset to data from block hdead
		addr += sizeof(struct FBitstreamBlockInfo);
		
		// Calc block & total crc
		int blockCrc = 0;
		for(int i=0;i<blockInfo->blockSz;i++)
		{
			crc += addr[i];		
			blockCrc += addr[i];		
		}

		blockCrc = blockCrc & 0xff;		
		
		if(blockInfo->blockCrc != blockCrc)
		{
			DEBUG_PRINT("blockInfo[%d]->blockCrc %d != blockCrcd %d\r\n", blockId, blockInfo->blockCrc, blockCrc); 
			
			return 0;
		}
		
		DEBUG_PRINT( "Verified Block[%d] sz: %d, crc: %d\r\n", blockId, blockInfo->blockSz, blockInfo->blockCrc );
	}

	// Validate final crc
	crc = crc & 0xff;
	
	if( crc != info->crc )
	{		
		DEBUG_PRINT(tmp, "crc %d != info->crc %d\r\n", crc, info->crc ); 		
		return 0;				
	}
	
	return 1;
}


/** Auto program bitstream from flash storage if valid.
*/
int auto_program_bitstream_flash( struct FPGA_config_t* config, int forceIfValid ) {
    
	DEBUG_PRINT("auto_program_bitstream_flash %d\r\n", forceIfValid); 
	
	struct FBitstreamFlashInfo* prev_info = find_bitstream_info_flash( );
	if(!prev_info)
	{
		DEBUG_PRINT("[Not found] auto_program_bitstream_flash\r\n");
		return 0;
	}
		
	DEBUG_PRINT("[FoundBitstream] blockCnt: %d, bitStreamSz: %d\r\n", prev_info->blockCnt, prev_info->bitStreamSz );
	
	int isValid = verify_bitstream_flash( prev_info );
	
	DEBUG_PRINT("verify_bitstream_flash isValid: %d\r\n", isValid); 
	
	int isBusy = 0;	
	if( isValid && (prev_info->programOnStartup || forceIfValid) )
	{		
		DEBUG_PRINT("Writing bitstream\r\n"); 

		isBusy = fpga_poll_busy( config );
		DEBUG_PRINT("auto_program_bitstream_flash isBusy: %d\r\n", isBusy);
		if(isBusy)
		{		
			DEBUG_PRINT("[Abort] isBusy: %d\r\n", isBusy);		
			return 0;
		}
		
		// Begin fpga program
		fpga_isc_enable( config );
		fpga_write_bitstream_begin( config );
	
		// Block loop
		int crc = 0;
		for(int i=0;i<prev_info->blockCnt;i++)
		{
			int pageId = 0; // Always use page0 in sector, no wear leveling
			int blockId = i + 1;
			uint8_t * addr = XIP_BASE + FLASH_TARGET_OFFSET + (blockId * FLASH_SECTOR_SIZE) + (pageId * FLASH_PAGE_SIZE);	
			
			struct FBitstreamBlockInfo* blockInfo = (struct FBitstreamBlockInfo*)addr;
			if(blockInfo->blockSz > FLASH_SECTOR_SIZE)
			{		
				return 0;
			}
			
			addr += sizeof(struct FBitstreamBlockInfo);
			
			int blockCrc = 0;
			for(int i=0;i<blockInfo->blockSz;i++)
			{
				crc += addr[i];		
				blockCrc += addr[i];		
			}

			blockCrc = blockCrc & 0xff;		
			
			if(blockInfo->blockCrc != blockCrc)
			{
				return 0;
			}
			
			// Write block to fpga
			fpga_write_bitstream_block( config, addr, blockInfo->blockSz );			
		}
		
		// End program
		fpga_write_bitstream_end( config );
		
		// Disable config mode
		fpga_isc_disable( config );

		// Check complete
		isBusy = fpga_poll_busy( config );
		DEBUG_PRINT("auto_program_bitstream_flash isBusy: %d\r\n", isBusy);
			
		return 1;
	}
		
	return 0;
}


void auto_end_program_cycle( struct FPGA_config_t* config )
{
	// End program
	fpga_write_bitstream_end( config );
	
	// Disable config mode
	fpga_isc_disable( config );

	sleep_ms(100);
							
}


int main() {	

	// init    
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);	
	 
    stdio_usb_init();	
	stdio_set_translate_crlf(&stdio_usb, false);
	stdio_flush();

#if ENABLE_DEBUG_LOG
	// Init debug uart
    uart_init(DEBUG_UART_ID, 115200);
    gpio_set_function(DEBUG_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(DEBUG_UART_RX_PIN, GPIO_FUNC_UART);
    int __unused actual = uart_set_baudrate(DEBUG_UART_ID, 115200);
    uart_set_hw_flow(DEBUG_UART_ID, false, false);
    uart_set_format(DEBUG_UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(DEBUG_UART_ID, false);
	DEBUG_PRINT("\r\n\r\n[DEBUG MAIN]\r\n");
#endif
		
	// init device
	struct FPGA_config_t config;
	fpga_init_config( &config, BOARD_ANY );

	// auto program on startup
	//auto_program_bitstream_flash( &config, 0 );	
	
	uint8_t isBusy ;
	int isProgramming = 0;
	int isSavingToFlash = 0;
	int flashCrc = 0;
	struct FBitstreamFlashInfo flashInfo;

	// Send startup error code & flash led	
	gpio_put(LED_PIN, 1);
	sleep_ms(200);
	gpio_put(LED_PIN, 0);
	
	struct FGeneric_Response response;
	response.header.cmd = FCMD_DeviceStartup;
	response.header.counter = 1;
	response.errorCode = 0;
	writeBlock( (uint8_t*)&response, sizeof(struct FQueryDevicePacket_Response));	
	
    while (1)
	{
		//gpio_put(LED_PIN, isProgramming);		
		
		int sz = readBlock( requestPacket, sizeof(requestPacket) );
		if(sz >= sizeof(struct FPayloadHeader))
		{			
			struct FPayloadHeader* requestHeader = ((struct FPayloadHeader*)requestPacket);
			
			DEBUG_PRINT("Dispatch[%d]: cmd: %d\r\n", requestHeader->counter, requestHeader->cmd);

			// Cmd dispatch
			switch(requestHeader->cmd)
			{
				case FCMD_Echo:
				{							
					// echo data back
					writeBlock( requestPacket, sz);
					break;
				}
				case FCMD_QueryDevice:
				{
					if(sz < sizeof(struct FQueryDevicePacket))
					{
						struct FGeneric_Response response;
						response.header = *requestHeader;
						response.header.cmd = FCMD_ErrorCmd;
						response.errorCode = 1; // Unkown cmd			
						writeBlock( (uint8_t*)&response, sizeof(struct FQueryDevicePacket_Response));
						break;
					}
					
					// Force end
					if(isProgramming)
					{
						auto_end_program_cycle( &config );
						isProgramming = 0;
					}
					
					struct FQueryDevicePacket_Response response;
					response.header = *requestHeader;
											
					// Get device id
					pico_unique_board_id_t* device_id = (pico_unique_board_id_t*)response.progDeviceId;
					pico_get_unique_board_id(device_id);

					// read device id
					uint32_t deviceId = fpga_read_id( &config );					
					response.deviceState = 0;
					switch(deviceId & 0x0FFFFFFF)
					{
						//case FPGA_DEVID_LFE5U_12:			
						case FPGA_DEVID_LFE5U_25:					
						//case FPGA_DEVID_LFE5U_45:					
						case FPGA_DEVID_LFE5U_85:					
							response.deviceState = 1;
							break;							
					};
					response.fpgaDeviceId = deviceId;							

					DEBUG_PRINT("FCMD_QueryDevice[%d]: deviceId: %d, progDeviceId: %X%X%X%X%X%X%X%X\r\n", requestHeader->counter, deviceId,
						response.progDeviceId[0], response.progDeviceId[1], response.progDeviceId[2], response.progDeviceId[3],
						response.progDeviceId[4], response.progDeviceId[5], response.progDeviceId[6], response.progDeviceId[7]
					);
			
					writeBlock( (uint8_t*)&response, sizeof(struct FQueryDevicePacket_Response));
					break;
				}
				case FCMD_ProgramDevice:
				{
					if(sz < sizeof(struct FProgramDevicePacket))
					{
						struct FGeneric_Response response;
						response.header = *requestHeader;
						response.header.cmd = FCMD_ErrorCmd;
						response.errorCode = 1; // Unknown cmd			
						writeBlock( (uint8_t*)&response, sizeof(struct FQueryDevicePacket_Response));
						break;
					}
					
					struct FProgramDevicePacket* requestData = ((struct FProgramDevicePacket*)requestPacket);
					
					DEBUG_PRINT("FCMD_ProgramDevice: saveToFlash: %d, totalSize: %d, blockCount: %d, bitstreamCrc: %d\r\n", 
						requestData->saveToFlash,
						requestData->totalSize,
						requestData->blockCount,
						(int)requestData->bitstreamCrc
						);
					
					// Init save info
					isSavingToFlash = requestData->saveToFlash;
					
					flashInfo.magic0 = FLASH_MAGIC_0;
					flashInfo.programOnStartup = 1; // TODO: give option, writing should assume load. Future use for button trigger
					flashInfo.blockCnt = requestData->blockCount;
					flashInfo.bitStreamSz = requestData->totalSize;
					
					// clear crc
					flashCrc = 0;
					
					isBusy = fpga_poll_busy( &config );
					DEBUG_PRINT("isBusy: %d\r\n", isBusy);
					if(!isBusy)
					{								
						fpga_isc_enable( &config );

						// Begin
						fpga_write_bitstream_begin( &config );
						
						// Set program flag
						isProgramming = 1;
					}							
					struct FGeneric_Response response;
					response.errorCode = (!isBusy) ? 0 : 1;					
					writeBlock( (uint8_t*)&response, sizeof(struct FQueryDevicePacket_Response));												
					break;
				}
				case FCMD_ProgramBlock:
				{						
					if(sz < sizeof(struct FQueryProgramBlock))
					{
						struct FGeneric_Response response;
						response.header = *requestHeader;
						response.header.cmd = FCMD_ErrorCmd;
						response.errorCode = 1; // Unknown cmd			
						writeBlock( (uint8_t*)&response, sizeof(struct FQueryDevicePacket_Response));
						break;
					}
					
					struct FQueryProgramBlock* requestData = ((struct FQueryProgramBlock*)requestPacket);
					
					DEBUG_PRINT("FCMD_ProgramBlock: blockId: %d, compressedBlockSz: %d, blockSz: %d, blockCrc: %d\r\n", 
							requestData->blockId,
							requestData->compressedBlockSz,
							requestData->blockSz,
							requestData->blockCrc
						);
					
					// Get ptr to data
					uint8_t* bitStreamBlock = requestPacket + sizeof(struct FQueryProgramBlock);
					
					// Decompress data
					//uint16_t raw_sz = bitStreamBlock[0] << 8 | bitStreamBlock[1]; // Used for allocation, as this is all fixed ignoring these values.
					uLong uncomp_len = sizeof(uncompressedData);
					int cmp_status = uncompress( (unsigned char *)&uncompressedData, &uncomp_len, &bitStreamBlock[2], requestData->compressedBlockSz );
					if( cmp_status != Z_OK)
					{
						DEBUG_PRINT("[Error] Decompress failed\r\n");
						
						struct FGeneric_Response response;
						response.header = *requestHeader;
						response.header.cmd = FCMD_ErrorCmd;
						response.errorCode = 1; // Unknown cmd			
						writeBlock( (uint8_t*)&response, sizeof(struct FQueryDevicePacket_Response));
						break;
					}
					
					if( uncomp_len != requestData->blockSz )
					{
						DEBUG_PRINT("[Error]uncomp_len %d != requestData->blockSz %d\r\n", uncomp_len, requestData->blockSz );
						
						struct FGeneric_Response response;
						response.header = *requestHeader;
						response.header.cmd = FCMD_ErrorCmd;
						response.errorCode = 1; // Unknown cmd			
						writeBlock( (uint8_t*)&response, sizeof(struct FQueryDevicePacket_Response));
						break;						
					}
					
					// crc bitstream
					int crc = 0;
					for( int i=0;i<requestData->blockSz;i++)
					{
						crc += uncompressedData[i];
					}					
					crc = crc & 0xff;
					
					if( crc != requestData->blockCrc )
					{
						DEBUG_PRINT("[Error]blockCrc %d != requestData->blockCrc %d\r\n", crc, requestData->blockCrc  );
						
						struct FGeneric_Response response;
						response.header = *requestHeader;
						response.header.cmd = FCMD_ErrorCmd;
						response.errorCode = 1; // Unknown cmd			
						writeBlock( (uint8_t*)&response, sizeof(struct FQueryDevicePacket_Response));
						break;						
					}
					
					// Write block to fpga
					fpga_write_bitstream_block( &config, uncompressedData, requestData->blockSz );
					
					struct FGeneric_Response response;
					response.header = *requestHeader;
					response.errorCode = 0; // OK
					writeBlock( (uint8_t*)&response, sizeof(struct FQueryDevicePacket_Response));
					
					// Save block to flash
					if(isSavingToFlash)
					{
						DEBUG_PRINT( "write_bitstream_block_flash blockId %d, blockCrc: %d\r\n", requestData->blockId, crc8_block( uncompressedData, uncomp_len ) ); 
						
						if(!write_bitstream_block_flash( requestData->blockId, uncompressedData, uncomp_len, &flashCrc ))
						{
							DEBUG_PRINT("[FAILED] write_bitstream_block_flash %d failed to write\r\n", requestData->blockId);
							
							// Clear save flag
							isSavingToFlash = 0;
						}
					}
						
					break;
				}
				case FCMD_ProgramComplete:
				{					
					DEBUG_PRINT("FCMD_ProgramComplete\r\n"	);
					
					// End program
					fpga_write_bitstream_end( &config );
					
					// Disable config mode
					fpga_isc_disable( &config );

					// Check complete
					isBusy = fpga_poll_busy( &config );
					DEBUG_PRINT("FCMD_ProgramComplete isBusy: %d\r\n", isBusy);
					
					struct FGeneric_Response response;
					response.header = *requestHeader;
					response.errorCode = (!isBusy) ? 0 : 1;
					writeBlock( (uint8_t*)&response, sizeof(struct FQueryDevicePacket_Response));	
					
					// Commit flash if success
					if( isSavingToFlash && response.errorCode == 0 )
					{
						flashCrc = flashCrc & 0xff;
						
						DEBUG_PRINT("write_bitstream_info_flash\r\n");						
						flashInfo.crc = flashCrc;
						flashInfo.bitStreamCrc1 = flashInfo.crc + 1;
						flashInfo.bitStreamCrc2 = flashInfo.crc + 2;
						if(!write_bitstream_info_flash( &flashInfo ))
						{
							DEBUG_PRINT("[FAILED] Info failed to write\r\n");
						}
					}
	
					isProgramming = 0;
					break;
				}		
				case FCMD_QueryBitstreamFlash:
				{
					// Force end
					if(isProgramming)
					{
						auto_end_program_cycle( &config );
						isProgramming = 0;
					}
					
					DEBUG_PRINT("FCMD_QueryBitstreamFlash\r\n"	);					
					
					struct FQueryBitstreamFlash_Response response;
					memset(&response, 0, sizeof( struct FQueryBitstreamFlash_Response ) );
					response.header = *requestHeader;
					response.errorCode = 1;	 // Default error
					
					// Read info from flash
					struct FBitstreamFlashInfo* info = find_bitstream_info_flash( );					
					if(info)
					{						
						// Verify bitstream
						int isValid = verify_bitstream_flash( info );
						if(isValid)
						{
							// Fill out flash info
							response.errorCode = 0;	
							response.programOnStartup = info->programOnStartup;	
							response.blockCnt = info->blockCnt;	
							response.bitStreamSz = info->bitStreamSz;	
							response.crc = info->crc;	
						}							
					}
					
					writeBlock( (uint8_t*)&response, sizeof(struct FQueryBitstreamFlash_Response));	
					
					break;
				}		
				case FCMD_ProgramBitstreamFromFlash:
				{
					// Force end
					if(isProgramming)
					{
						auto_end_program_cycle( &config );
						isProgramming = 0;
					}
					
					DEBUG_PRINT("FCMD_ProgramBitstreamFromFlash\r\n" );
					
					// Run startup program with force options
					int result = auto_program_bitstream_flash( &config, 1 );
					
					struct FGeneric_Response response;
					response.header = *requestHeader;
					response.errorCode = result == 0;
					writeBlock( (uint8_t*)&response, sizeof(struct FQueryDevicePacket_Response));	
					
					break;
				}	
				case FCMD_ClearBitstreamFlash:
				{
					// Force end
					if(isProgramming)
					{
						auto_end_program_cycle( &config );
						isProgramming = 0;
					}
					
					DEBUG_PRINT("FCMD_ClearBitstreamFlash\r\n" );

					// Clear info
					DEBUG_PRINT("zeroing flash header\r\n");						
					memset(&flashInfo, 0, sizeof(struct FBitstreamFlashInfo) );					
					
					// Erase
					struct FGeneric_Response response;
					response.header = *requestHeader;
					response.errorCode = 0;
					
					if(!write_bitstream_info_flash( &flashInfo ))
					{
						DEBUG_PRINT("[FAILED] Info failed to write\r\n");
						response.errorCode = 1;
					}
					
					writeBlock( (uint8_t*)&response, sizeof(struct FQueryDevicePacket_Response));	

					break;
				}
				case FCMD_RebootProgrammer:
				{
					// Abuse the watchdog
					#define AIRCR_Register (*((volatile uint32_t*)(PPB_BASE + 0x0ED0C)))
					AIRCR_Register = 0x5FA0004;
					
					DEBUG_PRINT("[??] Are we still alive?\r\n");
					break;
				}
				default:
				{
					struct FGeneric_Response response;
					response.header = *requestHeader;
					response.errorCode = 1; // Unkoqn cmd			
					writeBlock( (uint8_t*)&response, sizeof(struct FQueryDevicePacket_Response));
				}
			}

		}

        tight_loop_contents();
	}
}
