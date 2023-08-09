"""
Pico Fabric serial usb bitstream programmer. Uploads bitstream to fpga
using the Pico.

Device preparation:
    Install the bootloader.uf2 or bootloader_w.uf2 onto the Pico device.

Basic usage:
    $ program.py bitstream.bit

Options:
    Query device to validate fpga is working
    $ program.py --test

    Run blink to validate device, LED on fabric board should flash
    $ program.py --blinky 

    to determin the port if auto detection fails or multiple devices used.
    $ program.py --port=COM6 bitstream.bit

    Option to save the bitstream to flash
    $ program.py --save=1 bitstream.bit

Dependencies:
    pyserial
    
"""

# defaults
DEFAULT_FABRIC_PORT = 21560
DEFAULT_BAUD = 115200
IGNORE_PORTS = ['COM1']
PREFERRED_PROBE_PORTS = { 'Linux': ['/dev/ttyACM*'], 'Darwin': ['/dev/cu.usbmodem*'] } # auto probe check ports first
SERIAL_FAST_TIMEOUT = 0.1
SERIAL_NORMAL_TIMEOUT = 2.5

# imports
import os, sys, io, time, zlib, random, math, json, fnmatch, platform, traceback, base64
from optparse import OptionParser

try:
    import serial

    if os.name == 'nt':  # sys.platform == 'win32':
        from serial.tools.list_ports_windows import comports
    elif os.name == 'posix':
        from serial.tools.list_ports_posix import comports    
    else:
        raise ImportError("Sorry: no implementation for your platform ('{}') available".format(os.name))


except ImportError:    
    print("pyserial (https://pypi.org/project/pyserial/) module is missing\n enter the following into the CLI to install:\n$ pip install pyserial")
    exit(1)


# embeded blinky bits
blink_bits = """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"""\


# embed uf2 bootloader image
bootloader_uf2_image = \
"""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"""\
"""VyciA+7YQ9/t32b+Wfr/L4/wFhU="""\


class LogLevel:
    GlobalLevel = 'debug'
    AllLevels = ['data', 'debug', 'trace', 'info', 'warn', 'error', 'progress']
    JsonLogMode = False

    Data = 'data'
    Debug = 'debug';
    Trace = 'trace';
    Info = 'info';
    Warn = 'warn'
    Error = 'error'
    Progress = 'progress'    


def plural( w, c, possessive=False ):
    if c > 1:
        if possessive:
            return w + "'s"
        return w + "s"
    return w

def log( logType, msg, code=0 ):

    # json data filter
    if logType == LogLevel.Data:
        if LogLevel.JsonLogMode:
            data = {'t':logType }
            for k,v in msg.items():
                data[k]=v
            print(json.dumps( data ) )
            return
        else:
            return # ignore non json
    
    # filter level
    levelId = LogLevel.AllLevels.index( logType )
    globalLevelId = LogLevel.AllLevels.index( LogLevel.GlobalLevel )
    if levelId < globalLevelId:        
        return
    
    if LogLevel.JsonLogMode:
        print(json.dumps( {'t':logType, 'msg':msg, 'c':code } ) )
    else:
        print("[%s] %s" % (logType, msg) )


def formatException(e):
    """
        Exception formatter for logging.
    """
    exList = traceback.format_stack()
    exList = exList[:-2]
    exList.extend(traceback.format_tb(sys.exc_info()[2]))
    exList.extend(traceback.format_exception_only(sys.exc_info()[0], sys.exc_info()[1]))

    resStr = "Exception Traceback (most recent calls):\n"
    resStr += "".join(exList)    
    resStr = resStr[:-1]

    return resStr


def exitWithError( msg, code=1):
    log( LogLevel.Error, msg, code=code )
    sys.exit( code )


def dumpBytes( b ):
    res = ""
    for i in b:
        if res:
            res = res + ", "
        res += "0x%X" % i
    print(res)

    
def compressData( data ):
    """
        Compress with size header
    """
    sz = len(data)
    szBytes = bytes([
            (sz & 0xff00) >> 8,     # size[4]
            (sz & 0xff) >> 0,       # size[5]            
            ])    
    cdata = zlib.compress(bytes(data), level=9)
    
    return szBytes + cdata

    
def decompressData( data ):
    """
        Decompress with size header
    """
    sz = data[0] << 8 | data[1];
    return zlib.decompress( bytes( data[2:] ) )


class DeviceStatus:
    Unkown = 'unkown'
    StatusNoResponse = 'noresponse'    
    StatusExistsAndValid = 'ok'


class FabricDeviceInfo:
    """
        Fabric device info.
    """
    def __init__( s ):
        s.status = DeviceStatus.Unkown
        s.fpgaDeviceId = None # fpga device id
        s.uri = None # connection uri
        s.uid = None # pico uid

    def __repr__( s ):
        return "DeviceInfo( %s, %s, %s, %s )" % (str(s.status), str(s.fpgaDeviceId), str(s.uri), str(s.uid))


class FabricCommands:
    Echo = 0x00
    QueryDevice = 0x01
    UnkownCmd = 0xff
    ProgramDevice = 0x02
    ProgramBlock = 0x03
    ProgramComplete = 0x04
    QueryBitstreamFlash = 0x05
    ProgramBitstreamFromFlash = 0x06
    ClearBitstreamFlash = 0x07
    RebootProgrammer = 0x08
    
    
def _adduint8( a, b ):
    assert a >= 0 and a <= 0xff, "got " + str(a)
    assert b >= 0 and b <= 0xff, "got " + str(b)
    res = a + b
    if res > 0xff:
        res = (res % 0xff) - 1
    return res


class FEncoding:
    @staticmethod
    def getInt32( data, offset ):
        return (data[offset+0] << 0) | (data[offset+1] << 8) | (data[offset+2] << 16) | (data[offset+3] << 24)
    @staticmethod
    def decodeInt16( data, offset ):
        return (data[offset+0] << 0) | (data[offset+1] << 8)
    @staticmethod
    def encodeInt32( value ):
        return bytes( [ (value & 0xff) >> 0, (value & 0xff00) >> 8, (value & 0xff0000) >> 16, (value & 0xff000000) >> 24 ] )
    @staticmethod
    def encodeInt16( value ):
        return bytes( [ (value & 0xff) >> 0, (value & 0xff00) >> 8 ] )

    
class FCmdBase:
    def __init__( s, cmd ):
        s.cmd = cmd
        s.counter = 0
    def toBytes( s ):
        return bytes( [] )


class FResponseBase:
    def __init__( s ):
        s.cmd = None
        s.counter = None
        
    def fromBytes( s, data ):
        pass

        
class FQueryDevicePacket(FCmdBase):
    def __init__( s ):
        FCmdBase.__init__( s, FabricCommands.QueryDevice )
    def toBytes( s ):
        return bytes( [ 0 ] ) 


class FProgramDevicePacket(FCmdBase):
    def __init__( s ):
        FCmdBase.__init__( s, FabricCommands.ProgramDevice )
        s.saveToFlash = 0
        s.totalSize = 0
        s.blockCount = 0
        s.bitstreamCrc = 0
        
    def toBytes( s ):
        return bytes( [ s.saveToFlash ] ) + FEncoding.encodeInt32(s.totalSize) + FEncoding.encodeInt32(s.blockCount) + FEncoding.encodeInt16(s.bitstreamCrc)
    
    def __repr__( s ):
        return "FProgramDevicePacket( %s, %s, %s, %s )" % (str(s.saveToFlash), str(s.totalSize), str(s.blockCount), str(s.bitstreamCrc))


class FProgramCompletePacket(FCmdBase):
    def __init__( s ):
        FCmdBase.__init__( s, FabricCommands.ProgramComplete )
        
    def toBytes( s ):
        return bytes( [] )
    
    def __repr__( s ):
        return "ProgramComplete( )" 


class ClearBitstreamFlash(FCmdBase):
    def __init__( s ):
        FCmdBase.__init__( s, FabricCommands.ClearBitstreamFlash )
        
    def toBytes( s ):
        return bytes( [] )
    
    def __repr__( s ):
        return "ClearBitstreamFlash( )"
    

class RebootProgrammer(FCmdBase):
    def __init__( s ):
        FCmdBase.__init__( s, FabricCommands.RebootProgrammer )
        
    def toBytes( s ):
        return bytes( [] )
    
    def __repr__( s ):
        return "ClearBitstreamFlash( )"


class QueryBitstreamFlash(FCmdBase):
    def __init__( s ):
        FCmdBase.__init__( s, FabricCommands.QueryBitstreamFlash )
        
    def toBytes( s ):
        return bytes( [] )
    
    def __repr__( s ):
        return "QueryBitstreamFlash( )"

    
class FQueryProgramBlock(FCmdBase):
    def __init__( s ):
        FCmdBase.__init__( s, FabricCommands.ProgramBlock )        
        s.blockId = 0
        s.compressedBlockSz = 0
        s.blockSz = 0
        s.blockCrc = 0
        s.bitStreamBlock = bytes([])

    def toBytes( s ):
        return bytes( [] ) + FEncoding.encodeInt16(s.blockId) + FEncoding.encodeInt16(s.compressedBlockSz) + FEncoding.encodeInt16(s.blockSz) + bytes([s.blockCrc]) + s.bitStreamBlock
    
    def __repr__( s ):
        return "FProgramDevicePacket( blockId: %s, blockSz: %s )" % (str(s.blockId), str(s.blockSz))


class FGeneric_Response(FResponseBase):
    def __init__( s ):
        FResponseBase.__init__( s )
        s.errorCode = 0        
        
    def fromBytes( s, data ):                
        s.errorCode = FEncoding.getInt32( data, 0 )


class FQueryDevicePacket_Response(FResponseBase):
    def __init__( s ):
        FResponseBase.__init__( s )
        s.deviceState = 0
        s.fpgaDeviceId = 0
        s.progDeviceId = []
        
    def fromBytes( s, data ):        
        s.deviceState = data[0]        
        s.fpgaDeviceId = FEncoding.getInt32( data, 1 )
        s.progDeviceId = []
        for i in range(8):
            s.progDeviceId.append( data[ i + 1 + 4] )
        

class QueryBitstreamFlash_Response(FResponseBase):
    def __init__( s ):
        FResponseBase.__init__( s )
        s.errorCode = 0;
        s.programOnStartup = 0
        s.blockCnt = 0
        s.bitStreamSz = 0
        s.crc = 0
        
    def fromBytes( s, data ):                
        s.errorCode = FEncoding.getInt32( data, 0 )
        s.programOnStartup = FEncoding.getInt32( data, 4 )
        s.blockCnt = FEncoding.getInt32( data, 8 )
        s.bitStreamSz = FEncoding.getInt32( data, 12 )
        s.crc = data[16]

    def __repr__( s ):
        return "QueryBitstreamFlash_Response( errorCode: %s, programOnStartup: %s, blockCnt: %s, bitStreamSz: %s, crc: %s )" % (str(s.errorCode), str(s.programOnStartup),
                                                                                                                          str(s.blockCnt), str(s.bitStreamSz), str(s.crc) )


class FabricTransport:
    """
        Transport base class, provides high level
        device programming interface.
    """
    HeaderMagic = 0x1b
    MaxWriteBlockSize = 0xffff-16
    TransportTypeUSBSerial = 'usbserial'
    TransportTypeIP = 'ip'

    @staticmethod
    def createTransportForUri( uri ):
        """
            Create transport pipe for a serial or ip.
        """
        proto,port = uri.split('://')

        transport = None
        if proto == FabricTransport.TransportTypeUSBSerial:
            log( LogLevel.Debug, "Creating serial link for port %s" % port)
            transport = USBSerialTransport( FabricTransport.TransportTypeUSBSerial, uri, port=port )

        if not transport:
            return None
        
        transport.initTransport()
        return transport
            

    def __init__( s, transportType, uri, port=None, debug=0 ):
        s.uri = uri
        s.port = port
        s.transportType = transportType
        s.debug = debug
        s.init()

    def init( s ):
        """
            Init transport
        """
        # impl
        
    def writeCommand( s, timeout=None, responseClass=None ):
        """
            Write cmd to transport, optionally wait for a response.
        """
        # impl

    def setFastTimeoutMode( s, isFash ):
        """
            Option to use a faster timeout mode when scanning devices
        """
        # impl

    def queryDevice( s, timeout=None ):
        """
            Query device info            
        """        
        cmd = FQueryDevicePacket()        
        response = s.writeCommand( cmd, timeout=timeout, responseClass=FQueryDevicePacket_Response )
        if response:
            info = FabricDeviceInfo()            
            info.status = DeviceStatus.Unkown
            if response.deviceState == 1:
                info.status = DeviceStatus.StatusExistsAndValid
            info.fpgaDeviceId = response.fpgaDeviceId
            info.uri = s.uri
            info.uid = ''
            for i in response.progDeviceId:
                info.uid += hex(i)[2:]

            return info


    def programDevice( s, bitstreamData, saveToFlash=False, timeout=None ):
        """
            Program bitstream to device
        """        
        blockSz = 4096-32
        blockCnt = math.ceil(len(bitstreamData) / blockSz)

        sz = len( bitstreamData )
        
        # begin program
        cmd = FProgramDevicePacket()        
        if saveToFlash:
            cmd.saveToFlash = 1
        cmd.totalSize = sz
        cmd.blockCount = blockCnt
        cmd.bitstreamCrc = 0
        
        if s.debug > 0:
            log(LogLevel.Debug, "begin program cmd", cmd )
            
        response = s.writeCommand( cmd, timeout=timeout, responseClass=FGeneric_Response )        
        if response.errorCode != 0:
            print("Program Begin Device Response:", response)
            raise Exception("Device failed to program with code: %s" % str(response.errorCode) )

        # write blocks        
        i = 0
        blockId = 0
        while True:
            block = bitstreamData[ i : i + blockSz ]
            if not block:
                break
            blockSz = len(block)            

            # crc block
            blockCrc = 0
            for j in block:                
                blockCrc = blockCrc + j
            blockCrc = blockCrc & 0xff

            # compress block
            compressedBlock = compressData( block )
            
            # begin program
            cmd = FQueryProgramBlock()
            cmd.blockSz = len(block)
            cmd.compressedBlockSz = len(compressedBlock)
            cmd.blockId = blockId
            cmd.bitStreamBlock = compressedBlock
            cmd.blockCrc = blockCrc

            if s.debug > 0:
                log(LogLevel.Debug, str(cmd) + " %s, %s" % (str(len(block)), str(len(compressedBlock)) ) )

            # log progress
            log(LogLevel.Progress, "Chunk %s / %s" % (str(i), str(sz) ) )
            
            response = s.writeCommand( cmd, timeout=timeout, responseClass=FGeneric_Response )        
            if response.errorCode != 0:
                print("Write block device Response:", response)
                raise Exception("Device failed to program with code: %s" % str(response.errorCode) )
        
            i = i + blockSz
            blockId = blockId + 1

        # write end program and verify        
        cmd = FProgramCompletePacket()        

        if s.debug > 0:
            print("begin end cmd", cmd )
        
        response = s.writeCommand( cmd, timeout=timeout, responseClass=FGeneric_Response )        
        if response.errorCode != 0:
            print("Program End Device Response:", response)
            raise Exception("Device failed to program with code: %s" % str(response.errorCode) )

        log(LogLevel.Progress, "Completed %s / %s" % (str(sz), str(sz) ) )            

        return True


    def clearFlash( s, timeout=None ):
        """
            Clear flash and prevent bitstream boot.
        """        
        cmd = ClearBitstreamFlash()        
        response = s.writeCommand( cmd, timeout=timeout, responseClass=FGeneric_Response )
        if response:
            return response.errorCode == 0


    def rebootProgrammer( s, timeout=None ):
        """
            Reboot programmer device
        """
        # run cmd
        cmd = RebootProgrammer()        
        s.writeCommand( cmd, timeout=timeout, responseClass=None )

        return True


    def queryBitstreamFlash( s, timeout=None):
        """
            Query bitstream flash status.
        """
        # run cmd
        cmd = QueryBitstreamFlash()        
        return s.writeCommand( cmd, timeout=timeout, responseClass=QueryBitstreamFlash_Response )

    

class USBSerialTransport(FabricTransport):
    """
        Programs fabric over USB serial.
    """
    def init( s ):
        s.timeout = 10
        s.baudrate = DEFAULT_BAUD        
        s.counter = 0

    def initTransport( s ):
        """
            Low level re-init transport eg. recreate serial port etc.
        """
        s.ser = serial.Serial(port=s.port, baudrate=s.baudrate, timeout=s.timeout, write_timeout=s.timeout)
        s.ser.flushInput()
        s.ser.flushOutput()        

    def setFastTimeoutMode( s, isFash ):
        """
            Option to use a faster timeout mode when scanning devices
        """
        s.ser.timeout = SERIAL_FAST_TIMEOUT
        s.ser.write_timeout = SERIAL_FAST_TIMEOUT
        
    @staticmethod
    def writeBlock( ser, data ):
        """
            Write block with checksum
        """
        if len(data) >= USBSerialTransport.MaxWriteBlockSize:
            raise Exception("Max packet size")
        
        ser.write( bytes([ FabricTransport.HeaderMagic ]) + FEncoding.encodeInt16( len(data) + 1 ) ) # data + crc
        
        crc = 0
        for i in data:
            ser.write( bytes([ i ]) )
            crc = crc + i
            
        crc = crc & 0xff
    
        ser.write( bytes([ crc ]) )
    
    
    @staticmethod
    def readBlock( ser ):
        """
            Read block with checksum
        """
        data = []
        crc = 0

        # read magic
        raw_ch = ser.read(1)
        if not raw_ch:
            return None # timeout
        magic = raw_ch[0]
        assert magic == FabricTransport.HeaderMagic

        # read size
        raw_ch = ser.read(2)
        if not raw_ch or len(raw_ch) < 2:
            return None # timeout
        sz = FEncoding.decodeInt16( raw_ch, 0 )

        # read block
        for i in range(sz):
            raw_ch = ser.read(1)
            if not raw_ch:
                return None # timeout
            ch = raw_ch[0]
    
            if i < sz-1:
                crc = crc + ch
                
            data.append( ch )
    
        crc = crc & 0xff
        expected_crc = data[ len(data) - 1 ] & 0xff
    
        # verify crc
        if expected_crc != crc:
            raise Exception("Crc fail, got %d, expected %d" % (crc, expected_crc ))
    
        return data[0:len(data)-1] # remove crc


    def readPacket( s, timeout=0 ):
        """
            Read packet return cmd, counter, data
        """
        data = s.readBlock( s.ser )
        if data and len(data) >= 2:
            return data[0], data[1], data[2:]
        return None, None, None

    
    def writeCommand( s, cmd, timeout=None, responseClass=None ):
        """
            Write cmd and wait for response
        """
        s.ser.flushInput()
        s.ser.flushOutput()

        s.counter = _adduint8( s.counter, 1 )

        # create packet
        packet = bytes( [cmd.cmd, s.counter ] ) + cmd.toBytes() # FPayloadHeader + PayloadStruct
        s.writeBlock( s.ser, packet )

        # handle response
        if responseClass:
            return s.readCommand( responseClass )


    def readCommand( s, responseClass=None ):
        rcmd, rcnt, rdata = s.readPacket()

        if not rdata:
            raise Exception("No response")
            
        # instance and parse response
        responseCmd = responseClass()
        responseCmd.cmd = rcmd
        responseCmd.counter = rcnt
            
        responseCmd.fromBytes( rdata )
            
        return responseCmd
    
    
class FabricService:
    """
        Finds fabric devices on USB & IP networks.        
    """
    def __init__( s ):
        s.deviceCache = {}
        
    def listDevices( s, returnOnMinCnt=None, transportTypes=[FabricTransport.TransportTypeUSBSerial, FabricTransport.TransportTypeIP], useCache=True ):
        """
            Search transports like usbserial for valid devices.
        """
        devices = [];

        deviceUris = []        
        
        # Enumerate potential devices from com ports
        if FabricTransport.TransportTypeUSBSerial in transportTypes:
            serialPorts = comports(include_links=False)
            for n, (port, desc, hwid) in enumerate(serialPorts, 1):

                if port in IGNORE_PORTS:
                    continue
                
                # construct uri with usb serial port
                uri = FabricTransport.TransportTypeUSBSerial + '://' + port
                deviceUris.append( uri )


        # try fast preferred first
        prefList = []
        if platform.system() in PREFERRED_PROBE_PORTS:
            prefList = PREFERRED_PROBE_PORTS[ platform.system() ]
            
            for uri in deviceUris:
                isValid = False
                for i in prefList:                    
                    if fnmatch.fnmatch( uri, FabricTransport.TransportTypeUSBSerial + '://' + i ):
                        isValid = True
                        break                    
                if isValid:
                    deviceInfo = s.queryDevice( uri, fast=True )
                    if deviceInfo:
                        if not deviceInfo in devices:
                            devices.append( deviceInfo )
                        s.addDeviceCache( deviceInfo )
                        
                        # min cnt
                        if returnOnMinCnt != None and len(devices) >= returnOnMinCnt:
                            return devices
                    
                
        # query all / slow
        for fastMode in [True, False]:
            for uri in deviceUris:
                # query device for anything
                try:
                    deviceInfo = s.queryDevice( uri, fast=fastMode )
                    if deviceInfo:
                        if not deviceInfo in devices:
                            devices.append( deviceInfo )
                        s.addDeviceCache( deviceInfo )
                        
                        # min cnt
                        if returnOnMinCnt != None and len(devices) >= returnOnMinCnt:
                            return devices
                except:
                        pass
                    
        return devices


    def queryDevice( s, uri, fast=False ):
        """
            Returns device info
        """
        log(LogLevel.Debug, "query %s" % uri)
        
        # create transport with uri
        transport = FabricTransport.createTransportForUri( uri )
        if not transport:
            log(LogLevel.Debug, "Failed to create transport for '%s'" % uri)
            return None

        if fast:
            transport.setFastTimeoutMode( True )
            
        return transport.queryDevice()


    def addDeviceCache( s, deviceInfo ):
        if not deviceInfo or not deviceInfo.uri:
            return None
        s.deviceCache[ deviceInfo.uri ] = deviceInfo
        
    def queryDeviceOrGetCached( s, uri, fastMode=False ):
        if uri in s.deviceCache:
            return s.deviceCache[ uri ]
        
        transport = FabricTransport.createTransportForUri( uri )
        if not transport:
            log(LogLevel.Error, "Failed to create transport for uri '%s'" % uri )
            return 0
        
        deviceInfo = s.queryDevice( uri, fast=fastMode )
        if deviceInfo:
            s.addDeviceCache( deviceInfo )

        return deviceInfo


def embedBitstreamFromFile( f ):
    data = compressData( open(f,'rb').read() )
    encoded = base64.b64encode(data)

    blockSz = 90
    # write blocks        
    i = 0
    blockId = 0
    while True:
        block = encoded[ i : i + blockSz ]
        if not block:
            break
        blockSz = len(block)           

        print( '"""' + block.decode('iso-8859-1') + '"""\\')
        i += blockSz


def decodeEmbededBits( s ):    
    data = base64.b64decode(s)
    data = decompressData(data)
    return data


def writeBootloader( targetDir ):
    destPath = targetDir + "/" + "bootloader.uf2"
    log( LogLevel.Info, "Writing bootloader to %s'" %  destPath )

    open(destPath, "wb").write( decodeEmbededBits( bootloader_uf2_image ) )

#
#
def main():
    
    parser = OptionParser()
    parser.add_option("-t", "--test", action="store_true",
                      help="Test fabric device is working and identify fpga")
    parser.add_option("-q", "--quiet",
                      action="store_false", dest="quiet", default=False,
                      help="Don't print detailed status messages")
    parser.add_option("-p", "--port", dest="port",
                      help="COM port to use (instead of auto detection)")
    parser.add_option("-c", "--clearflash", action="store_true",
                      help="Clear bitstream flash and prevent bitstream load on startup")
    parser.add_option("-b", "--blinky", action="store_true",
                      help="Program test blinky to device to see if its working.")
    parser.add_option("-s", "--save", action="store_true",
                      help="Save bitstream to flash when programming device")
    parser.add_option("-j", "--json", action="store_true",
                      help="Echo output as json for automation parsing")
    parser.add_option("-r", "--rebootprogrammer", action="store_true",
                      help="Reboot programmer device")
    parser.add_option("-w", "--queryflash", action="store_true",
                      help="Query bitstream flash")
    parser.add_option("-v", "--bootloader", action="store_true",
                      help="")
    parser.add_option("", "--writebootloader", 
                      help="Install bootloader UF2 to image, allows PicoFabric IDE to program the FPGA via the Pico microcontroller. "\
                      "Set this value to the drive or mount point of the Pico device when in Bootsel mode to write to. eg. F:/ or /media/user/RPI-RP2 etc.")
        
    (options, args) = parser.parse_args()


    uri = None
    service = FabricService()

    if options.writebootloader:
        writeBootloader( targetDir=options.writebootloader )
        sys.exit(0)
        return

    # set log level
    if options.quiet:
        LogLevel.GlobalLevel = LogLevel.Warn
    if options.json:
        LogLevel.JsonLogMode = True
        
    # device selection
    if options.port:
        uri = FabricTransport.TransportTypeUSBSerial + '://' + options.port
    
    else:
        # auto detect        
        devices = service.listDevices( returnOnMinCnt=1) # find 1 device max
        log( LogLevel.Info, "Found %d %s" % (len(devices), plural('device', len(devices) )) )

        if devices:
            uri = devices[ 0 ].uri
            log( LogLevel.Info, "[Auto select] Using device '%s'" % uri )

    # main actions
    if options.test: # test device and print info
        if not uri:
            exitWithError( "No device found" )
            return 1
            
        log( LogLevel.Info, "Testing device at '%s'" %  uri )

        deviceInfo = service.queryDeviceOrGetCached( uri )        
        if not deviceInfo:
            exitWithError( "Failed to get device info for uri '%s'" % uri )
            return 1
        
        log( LogLevel.Info, "status: %s" % str(deviceInfo.status))
        log( LogLevel.Info, "fpgaDeviceId: %s" % str(deviceInfo.fpgaDeviceId))
        log( LogLevel.Info, "uid: %s" % str(deviceInfo.uid))

        # exit with status if not programming bit stream
        isDeviceOk = 0
        if deviceInfo.status == DeviceStatus.StatusExistsAndValid:
            isDeviceOk = 1
        log( LogLevel.Info, "deviceOk: %s" % str(isDeviceOk))
        

        log( LogLevel.Data, { 'status': deviceInfo.status,
                              'fpgaDeviceId': deviceInfo.fpgaDeviceId,
                              'uid': deviceInfo.uid   
                            } )

        if not args:
            if deviceInfo.status == DeviceStatus.StatusExistsAndValid:
                return 0
            else:
                exitWithError( "Device failed to detect FPGA" )
                return 1            


    transport = None
    if uri:
        # create transport with uri
        transport = FabricTransport.createTransportForUri( uri )
        if not transport:
            exitWithError("Failed to create transport for '%s'" % uri)
            return None


    if options.rebootprogrammer:
        log( LogLevel.Info, "Resetting programmer device '%s'" %  uri )
        
        if not transport.rebootProgrammer( True ):
            exitWithError( "Failed to reset programmer device '%s'" % uri )
            return 1
        
        log( LogLevel.Info, "Programmer device '%s' rebooted, exiting!" %  uri )
        return 0 # Cant do anything as transport link will go down
        
    if options.clearflash:
        log( LogLevel.Info, "Clearing flash on device '%s'" %  uri )

        if not transport.clearFlash():
            exitWithError( "Failed clear bitstream flash on device '%s'" % uri )
            return 1
        
        log( LogLevel.Info, "Flash cleared on device '%s'" %  uri )        

    if options.queryflash:
        log( LogLevel.Info, "Query flash on device '%s'" %  uri )

        flashInfo = transport.queryBitstreamFlash()
        if not flashInfo:
            exitWithError( "Failed query flash status on device '%s'" % uri )
            return 1

        hasValidBitstream = 0
        if flashInfo.errorCode == 0:
            hasValidBitstream = 1
            
        log( LogLevel.Info, "hasValidBitstream: %s" % str(hasValidBitstream))
        log( LogLevel.Info, "programOnStartup: %s" % str(flashInfo.programOnStartup))
        log( LogLevel.Info, "blockCnt: %s" % str(flashInfo.blockCnt))
        log( LogLevel.Info, "bitStreamSz: %s" % str(flashInfo.bitStreamSz))
        log( LogLevel.Info, "crc: %s" % str(flashInfo.crc))
        log( LogLevel.Data, { 'hasValidBitstream':hasValidBitstream,
                              'programOnStartup': flashInfo.programOnStartup,
                              'blockCnt': flashInfo.blockCnt,
                              'bitStreamSz': flashInfo.bitStreamSz,
                              'crc': flashInfo.crc,
                            } )
        
        
    if options.blinky:
        log( LogLevel.Info, "Uploading blinky bitstream to '%s', is saving: %s" % (uri, str(options.save)) )
        
        if not transport.programDevice( decodeEmbededBits( blink_bits ), saveToFlash=options.save ):
            exitWithError( "Failed program blinky bitstream on device '%s'" % (uri) )
            return 1
        log( LogLevel.Info, "Blink programmed on device '%s'" %  uri )
        
        
    if args:
        bitstreamFilename = args[ 0 ]

        if not uri:
            exitWithError( "No device found" )
            return 1
                    
        log( LogLevel.Info, "Uploading bitstream '%s' to '%s', is saving: %s" % (bitstreamFilename, uri, str(options.save)) )

        bitstreamData = open( bitstreamFilename, 'rb' ).read()

        if not transport.programDevice( bitstreamData, saveToFlash=options.save ):
            exitWithError( "Failed to program bitstream on device '%s'" % uri )
            return 1


if __name__ == '__main__':
    
    try:
        main()
    except Exception as e:
        # log exception        
        exitWithError( formatException(e) )

