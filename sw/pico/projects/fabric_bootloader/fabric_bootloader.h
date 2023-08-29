#pragma once

#include <stdint.h>
#include "../libfabric/libfabric.h"

/** Config
*/
#define ENABLE_DEBUG_LOG 0

/** Constants
*/
#define FPACKSTRUCT  __attribute__((__packed__)) 
#define FPacketHeaderMagic 0x1b


/** Serial commands
*/
enum FabricCommands
{
	FCMD_Echo = 0x00,				// Echo data back part of self test
	FCMD_QueryDevice = 0x01,		// Query device info, res
	FCMD_ProgramDevice = 0x02,		// Program device, multi part block data.
	FCMD_ProgramBlock = 0x03,		// Bitstream data block
	FCMD_ProgramComplete = 0x04, 	// End of bitstream, commit and verify
	FCMD_QueryBitstreamFlash = 0x05,// Get bistream info size, crc etc
	FCMD_ProgramBitstreamFromFlash = 0x06,// Program bitstream stored in flash
	FCMD_ClearBitstreamFlash = 0x07, // Clear bitstream flash on boot
	FCMD_RebootProgrammer = 0x08,  	// Reboot programmer device
	FCMD_DeviceStartup = 0xfe,  	// [non-disaptched] Sent on device startup
	FCMD_ErrorCmd = 0xff,			// Bad cmd
};


/** Header for data payload, contains cmd & counters.
*/
struct FPACKSTRUCT FPayloadHeader
{
	uint8_t cmd;
	uint8_t counter;
};


/** Generic Packet data.
*/
struct FPACKSTRUCT FGeneric_Response
{
	struct FPayloadHeader header;
	uint32_t errorCode;
};



/** FCMD_QueryDevice Packet data.
*/
struct FPACKSTRUCT FQueryDevicePacket
{
	struct FPayloadHeader header;
	uint8_t reserved;
};


/** FCMD_QueryDevice response data.
*/
struct FPACKSTRUCT FQueryDevicePacket_Response
{
	struct FPayloadHeader header;
	uint8_t deviceState;		// Device state eg. in user mode or ready.
	uint32_t fpgaDeviceId;		// FPGA device ID eg. DEVID_LFE5U_12	0x21111043
	uint8_t progDeviceId[8];	// Programmer uid
};


/** FCMD_ProgramDevice Packet data.
*/
struct FPACKSTRUCT FProgramDevicePacket
{
	struct FPayloadHeader header;
	uint8_t saveToFlash; 	// Store in pico flash
	uint32_t totalSize; 	// Total size
	uint32_t blockCount; 	// Number of blocks going to send
	uint16_t bitstreamCrc; 	// crc16 of bitstream data
};


/** FCMD_ProgramBlock Packet data.
*/
struct FPACKSTRUCT FQueryProgramBlock
{
	struct FPayloadHeader header;		
	uint16_t blockId;
	uint16_t compressedBlockSz;
	uint16_t blockSz;
	uint8_t blockCrc;
};


/** Flash bitstream info, uses lots of magic codes as flash state can be random data.
*/
struct FPACKSTRUCT FBitstreamFlashInfo
{
	uint32_t magic0;	 		// Write magic code to determin if flash has ever been writen
	uint32_t programOnStartup;	// programOnStartup
	uint32_t blockCnt;			// Total blocks used
	uint32_t bitStreamSz;		// Total size
	uint8_t crc; 				// write crc multiple times as flash will contain random data
	uint8_t bitStreamCrc1; 		// crc0+1 when valid
	uint8_t bitStreamCrc2; 		// crc0+2 when valid
};


/** Flash bitstream info, uses lots of magic codes as flash state can be random data.
*/
struct FPACKSTRUCT FBitstreamBlockInfo
{
	uint32_t blockId;
	uint32_t blockSz;
	uint8_t blockCrc;
};


/** Flash query response
*/
struct FPACKSTRUCT FQueryBitstreamFlash_Response
{	
	struct FPayloadHeader header;		
	uint32_t errorCode;
	uint32_t programOnStartup;	// programOnStartup
	uint32_t blockCnt;			// Total blocks used
	uint32_t bitStreamSz;		// Total size
	uint8_t crc; 				// write crc multiple times as flash will contain random data	
};
