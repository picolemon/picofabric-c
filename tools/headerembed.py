import sys

def generateCppEmbedFile( inputDataFilename, varname ):
    """
        Convert file to c include.
    """
    dataStr = open( inputDataFilename, "rb" ).read()
    assert(len(dataStr))

    codeStr = "#include <pico/platform.h>\nuint8_t __in_flash() %s[] = {\n" % varname

    bufferStr = ""
    maxCol = 32
    colCnt = 0
    for i in dataStr:
        charInt = (i)
        charValStr = str(charInt)
        if bufferStr:
            bufferStr = bufferStr + ", "
        colCnt = colCnt + len(charValStr) + 2
        if colCnt > maxCol:
             bufferStr = bufferStr + "\n"
             colCnt = 0
        bufferStr = bufferStr + charValStr
    codeStr = codeStr + bufferStr
    codeStr = codeStr + "\n};\n"
    codeStr = codeStr + "int %s_size = sizeof(%s);\n" % (varname, varname)
    return codeStr


def generateEmbededFileHeader( inputFilename, outputEmbedCppFilename, varname="embdedData" ):
    """
        generateEmbededFileHeader
    """   
    codeStr = ''    
    codeStr = codeStr + generateCppEmbedFile( inputFilename, varname ) 
    open(outputEmbedCppFilename,"w").write(codeStr)


generateEmbededFileHeader(sys.argv[1], sys.argv[2], sys.argv[3])

    



