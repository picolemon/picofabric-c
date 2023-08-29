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
blink_bits = """ifh42u2dCVxTV/bHT1K0BFHCoqIChh1cI4v/iEgTQEBUgi24FS3ggls1Ki5QbR8RKLIJaC1i/g6DViNY27pUx7ZjRG"""\
"""2to6IFHG2dGpAqolPBXdup86yfv/c8/w2DM2rr9PD5RPu9797zXu4993fOucHmHkQnzEsOkI0IG+of23eAT1jfgSGq"""\
"""4HCVj/9AuHfv3mfb+T/uDQaAev7lbC0NceH/VvKvMP4l5l9DX0iLjc5qZ7+70DEbHv684HuPwZFGAgICAgICAgICAg"""\
"""ICAoInDuLohxTa/xGi+SEgICAgeL5BBLMqaEYICAgISGIJCAgISGIJCAgICEhiCQgICAgICAgIKIklICAgICCJJSAg"""\
"""ICCJJSAgICCJJSAgICAgiSUgICAgiSUgICAgiSUgICAgIIklICAgICAgICCgJJbg1wX6P34SPCF/eZqu9CvdlnSWHJ"""\
"""UclYA8mKSWfJMclYA8mDyYpJYclaSWgDyYpJYclRyVgDyYPJik9ql8Cd2T/oK6Z3kvApLbp6F7bfZT0ltyTtJbAvJi"""\
"""8mKSXHJWclaCZ5LikhMTkOCSrxKQF1OOS85KzkpAXkxeTJJLzkpAQF5MkkvOSs5KQF5MkkuSS0BAQEASS0BAQEASS0"""\
"""BAQEBAEktAQEBAEktAQEBAQEBAQEBASSwBAQEBSSwBAQEBSSwBAQEBAUksAQEBwROA4vv/GcG/5kx7Qh0f/ITC1fdN"""\
"""kelRJTJI2MFwnx5qE9oybrUU9oU+rZnKsPOBLtUPWeRtOYD7fjFjORzey3qnnwKNrC1me7S3hBztL18U2WfKe/ubGL"""\
"""iPA3c2KyK3dJnbFHa1IAx2DkII2WWmyBpy9YyKpKsqZ5m62JqZIllxYmjburZGB/i35W7qYhuXKisdFLUmrkm55frH"""\
"""NmhTCZt6PEFfsg2F7e0Ymvlyy/qbvFtrNzdXeBu8HJDbqRQb/4Que8LiMybxkc4RKsXmYQwzX5YYdvVmzKkUHk0mbe"""\
"""0JlhjaJ5q8HKlSeCe29bnMjM2NbSPzIdHvvfQQxZJGtYMtJ49kuyJw+rll5brUL4UttfDG27jleBRY3xSacdKv+1Mr"""\
"""hvkWVw76uAi62EH1q62PSbb00JyQCVrsJ8OELrglioOKoYIbVSbBjYOopaVe4QnTC1t9GCl49BdwFOR8ZbpBNMU/B8"""\
"""TIy7q6wbVXkOCPG5loiLNi/d/RTAhf75RQGLXoIhvkmQtcCR4UtX7ZMHfjRWfWxdUNNAaGfvafwvmeyNdXgsqRoTg+"""\
"""6RJabeU3Xqb2gelLwo0HUCGYbK1m45TW5qlF7wncR6xBLKmu8uo8j03EEKk6Ga6yFYRsSd2eFG7wGGxVuzgBfvwztn"""\
"""L8GKz8DFuJtYRzp5iVHFVD1yhYfw0/ypZMDUQWYitfGs75YSO870R2ZEZUARbcmHLBgyw4w82NFj5a6br4m6PxfY62"""\
"""X1B9GU4vFNw7XatJekcwMFWmSzklaFkyQ3Iegqv/Ra8UvqUBPWTQoKxjoP8UbUiNOhS+WsIGmRWo84cpNt8WmnGGHz"""\
"""9AZobw85U4XmDFDWxSBFYcfLkh7QRWkr05dQSywo+S3+rMrFSGlzbH1cCuZjRqsnZxfQvI9woMzQ8FF7QDrcRaL094"""\
"""+Rjucyp1oa1iyotIzGRXWRYlkkI2mwQYWTjMUlZshyLxIVjriXADzGf3g7Ai2f5jaDv7jp0Im9kKwrpgaV+WZYjWaC"""\
"""aExQaOizHufpM1lr/iV71TKYk0kYxM8a8bupKb1p5dduCgNxMm8NHXQ/XX6PIYlXTkh4xjgqFsOkPt0IDOvpyV9Jdv"""\
"""9/MSG3Upl3CLc2MSnD2KllSjtlLC7iXYU0e2T+aq2NqIgvwLqpxsFZtSUa4kjapJAY9tAkMZXI0crd//2NUsHfv1tL"""\
"""komLwuVWfM9eW0KHYsGb+iInNhkNfdMDbUssKiKXXz+cOboNMP6MEaPtbAIm82tGnSHquKTFUTZDK1hwTfukEjrnF2"""\
"""W9lAc1ftYqOudAdqOVeZlC8xTBRuq8XGBphti95BV8fjS8caW5Q3/4z7VSatd9Y1zEaPm6NtTP0gsVO2Zl46Gj2GHx"""\
"""03s/CGMu0CGm6fe/eAAxd7BjuN/u7uBicvXzabopcGmN05ArF/FD7fJf3+UCReFns2t8i6VqO9dyhJruE6TxBoa1gw"""\
"""9NgnsHPaXX/l7MMW51xOu8QAhSVYOr/z4/deNzatZjbqvMn+o7I4CRNHM/vc66p9ttzWBQ+bXAo4bcokWHyH3a9TSD"""\
"""f3ZQWg3Sx8K/I1hlm1yAtiT5dN78plMKmFbl0TYASqFibHnl6u3si51eA9OC4stsanNL6UDYs67A4LZyDLEzOK+igd"""\
"""6lmLbI9cNhplZavjawei+Mm1qzAR7UC5CsYs+IVg/YuoLIAqJB/m3Gm9iWjLT4i7VuOc2kpyEZPbS77mCOpwznUDNL"""\
"""KsQNzOXu0drJmQhLZo8/3s6UpXpOdFXuD3HTLSXJbDveWLtsWAQ55/VPTLQR7VPEmhM266iowUqB2VGTWCR9kNMiTc"""\
"""5nptamcoK2BjLCS3sjzhc0+BlaHVXMAtgafyLW/ewvv9QFKZBbe3L9amQ55HZ+tS2+OmA7ffq+aWxrOHDg9yNTPW6w"""\
"""SJZLFKUZuL1qpIYuiDkjXPgwO6GYwOeIibNkU5LkxgpDwCjNfRoILivxw2afO+iT2aj2YLLHjBq6+ayrk8V/OxvydS"""\
"""6042y5P77V++Ln7pZYGRsslrZOK7JkS/U4fuk3M9NOfeFdjZUbyjm0GNahG+ZhajONrdyftD2J/NlsPSe775PNk/UD"""\
"""41yNJH6ZHBeoijQiJOyCAQZRDKTZZwtiNWNv/mncqerDAQOVjuL00HqxyUBofUTO+zspduchJWg3e+hfFIDkV2NpD5"""\
"""Lu7QdNvvbmM/VBDZRMK71sj726/MhMurUGj1usoFnMbbXhrDshRI25gct7qn4s4c1uT88SC4PhzP/LB9k305wwDcFK"""\
"""bVeHyOQ2X7zjJIGMJEsstHETYqxYsqvGHSjoRBMUuaRd3bOXVUXipHm4EfZcjNY2M6tIwcunxukJcv9vxz2l7W3Ni1"""\
"""SCLHZ+e+XQuLmEaJTsxPiSoMcdF9HYdC9Wq9VvPRCRSqFnWsiT/fBSRytPMdeQlKHIQ36LQileLaSPzu0/LqfLm1et"""\
"""xpStznIYFG3wmCQPh+RkJQ6bJiYVrbV/fXb5CT/mXl4aTzAU5bbl4UDK1JWj/ghnIc3gENy3bwWy3GBTtaaVUenwgc"""\
"""Y7k9nJev9l9VpZiLt/A5dTrXZxQ2pYm4DJ+8xUa9ypcAMDWTNay3XlHE50wnceZ+rnzLjCXlmhdYKuYsVbtJD8G3KC"""\
"""vP7tRpy0bNLpYsm7WTThvbYMtNeJk1SdXdU7lDMSyAStXL3dPBZiqeqpUW675VlkzGTS5HkqCOlXmirdUDD6xrztjP"""\
"""bu+xJ3rn66zD0gBLmz4w/DNB8Ax3jlmzyvhjCJLO0d2KocMKHD7z3bP7Kz1Qhi8dbl84He00yzxZ2jZT5109AHqvN3"""\
"""WQdAB85abKUUG4NFmO3i/c+dApNh06Ie8lH/DDobNBHVIKfxBIanLKUdkW9BY9yySG/t8gG/XRq250FtjY4gkgyGn4"""\
"""imv7RGTimEz3GgpKeUEfwu4TAhN5+xXjBRmeWhd3Cv7AchOz+1FR3n0ltjJoltEnAcV9SeNGTTyLteKUc/PsVIrbw7"""\
"""DdASVJsMZVWGjWSwxf7cDpe2xutUJ6Bj8gX3mui08eh8vlRq1m0A9IJIyjzIz/CBFIghI2ZCJBsBJnJ0HvF5GR9hq4"""\
"""NReXnXbHoH6OUBskyjwpMsK/yRQuCb0nUZAGImxQ5Dkp020rwjYOJW+pl9qw2km0L3rX2MrFsjVsjcRQFl0lqMgjeT"""\
"""UfiDQpe/mOElkjjPMzUdBZZ7tl3Q1gXCYb7jWnknUeIo52+wT5hBs0o8VUFsNbC/AeHl9yBWLR0fE6v9ISlDQValJC"""\
"""gpt8vA3avlj6BvYemVo6aCQqiuPnXxLUpDv+Wh4VhyrfRB9OzDI4+N+DR7h7LNiAokL58mqTWa9vXGbgm5q/72It7r"""\
"""BDcCQ0PMxVEW6NWxKiVQqtDIfJPslRkF2KpnqS5fEYGHoFeWR+EZ80zkpEGYfV6Ky+0L8TjpNVSlCgwz+rVyxrEr6Y"""\
"""WQuOqP4zK7ztcKAL52tAkzJUf9v++/VNUY5Me0W18ye075y3oFv02VgUeSd9HBUPHuE4Fo/71HLRqBvKjeiwPsF/Pq"""\
"""87epaRi2KlmlUqRfVr6D0GZw2xkDXAtoHoHdir24XacjEDhWXn6xcW9LRbeRuPzUnplygx6N5EbRUlTSkjZlY0bYI6"""\
"""HM27OtQkfbGgp1+LDV6DZQZe1JLuoNOIgBWq/NNNXAvbAlC4b1SycdaXONYtG1UKV44KtmeA8wFDzhq8hR1eM4eFKJ"""\
"""8uvB9Gwz8WJCtLpkZcgD2LsKXYUNm20bju3D9g7Sb4GpVzhd+dmGU8f1CQPYXKSvHxbsvmKeWys3WsxUK/wm6aP1xL"""\
"""EzhlPp+qZBbjiDj0SHbzJhsUySQFh8DsfTxDlvrLIK7HW3BcuKtVbGFUGjpo6lWnkM3YhC3nvzdyUWn3fqjmhF5ogH"""\
"""eIvYtTWw5blTBjK5IPUOhNUds+NvnXx9jiThba0PhV6HO43KJiZawzjgv5KV66wV8gFQwsiAJHvNiNha420A994ph7"""\
"""xBtu+gqin4umy+LWiie1hyXMEyMTW+FAhsCCmy/XYsEew/n+Y3QeKLDhZstt926txUKvjrPl1ke2eqSt7lvNbS96PM"""\
"""P/r4WvMWW6s/fwpyYNHAycjnQq00t3MlBoQnatGE/SxfMQPBObuNaByx+BTKQvN1ifFNSgGfJtfzc5z6LAeVWblXAt"""\
"""GJXvG3qv1cIlJ4ERcOqD68k0M+OtZaghp3tJjIO8MQbFoNK5M1F56WaYi47Zy4JLYkpn16OncobvV6JyL0jLXUCqpD"""\
"""y+TXnGC6m3ZmfRDG4M+rB23ZL1fF3ZqMV15SmPOp/SmxIs+mOrlU5KlKqO1sVNxZEyzt1ojY5fVW5cMjpeFtVftI6A"""\
"""E0UoUnrVcvGoqBx/xBneRCfOaauTHbt8oAhHqaVPZgEsfRtP7pTeScBdwS3BvAPivKRlg9cFkKLwFeQNMGUPWvbZZV"""\
"""lweRSqKCV8NA3BhymNH6f20sWyMCGqry/JcTrVH7YPwHZssmRQPQGXlLnX5wXpJ0nQan/QvdHPthwCUEEeuJqLRJrc"""\
"""Qf63SafKOkanNeIgwStNvb8gJGWUmYNnEn6nxYPHy9fAjgjUdifL84u+un5aHOUdalI7zRzWory0Cz3DbIstlZq9W4"""\
"""UfsXk7nI4xXkdT01S386p9zE/67ug0xDNfk7cHTU3d9iXWnykBiUvg3KqeqkBjKjsPEB0KO5iQr9mNEojAWb26LS7Q"""\
"""6/AGbS65A9+jLC/IptG1XLO8CfWZf1itmFiB30i/QRdgMDprCbKv/3CDcetB1GdOr80XIB7Vc4EJijdOoyF2q3SOZ1"""\
"""o5Unyk/yMfX4rEoT4GGG1uusM7Icc1u4f3AsdDpo5A75flF2zCE+Gn7qwNIkGcigrOor/Zj641eUzatipQCMICtK2X"""\
"""2gjRkP3+M/m1Ih2Umfi1DRHAjL30a1dtA3PD2N6Pf6kVe238mPw/Nye8FB0MLgHPfvo2fgjifFP78ZGLz2zSrDmriv"""\
"""9m+remQXDJfkw0vHHcVFe7k3B20nO4l6Wfw17fJ9qRgIDgeYZsJciuPtGO9I9pCAgICOjfKxIQEBCQxJLEEhAQEPzO"""\
"""JVbCvybqae0ICAhIY3/vU6q4rHiS/QieV5CCRYUpovkhmSUgICAgIIkleB5ydtMghqmP735i5QUvU0TL8iuBWD61w7"""\
"""MRqdboaephW+8ksp/1w79h3fQoAgICAgICAgICAjozICAgICCJJSAgICAgiSUgICAgiSUgICAgiSUgICAgIIklICAg"""\
"""IIklICAgIIklICAgICD47YE4+iGF9v+VidaDPI+8hFLaVnxG85A+T2/zpd+ce5OfEfwWRZhmhHSWgICAgCSWUtnHfG"""\
"""6C35crEBBQKktA8Ls6LhDJf8YPdj8npwf/qSq18lUqT/hbVtp8J/p2FwICAgICAgICgv8+KFDamItVECGTmIlknIsk"""\
"""XSVvtnukT5Sr5ECCItfTOr1erguzyQoeZb7b85E+xaG25toE6YifLWS5Sc20YPzpkT7p7tL0Mumq4J8tRNpmVc6193"""\
"""B/3mfRphI29XgKxkUPaOYnrc0b6tSWBfi/hZQ9WMv7FpWBXR5YenAIeqnKlEvgn0rugc2srIn8n/db/gmVv4yP"""\


# embed uf2 bootloader image
bootloader_uf2_image = \
"""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"""\
"""0qYLUbvt3f7v6j9P9vNcP2iQ=="""\


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

