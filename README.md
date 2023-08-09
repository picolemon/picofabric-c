# PicoFabric C SDK :lemon: # 
[![Install](https://img.shields.io/badge/VSCode-Extension-f3cd5a?longCache=true&style=flat-rounded)](https://github.com/picolemon/picofabric-ide)
[![datasheet (pdf)](https://img.shields.io/badge/Data%20Sheet-PDF-f3cd5a?longCache=true&style=flat-rounded)](https://github.com/picolemon/picofabric-hardware/blob/main/doc/datasheet.pdf)
[![sch (pdf)](https://img.shields.io/badge/SCH-PDF-f3cd5a?longCache=true&style=flat-rounded)](https://github.com/picolemon/picofabric-hardware/blob/main/doc/sch.pdf)
[![Store](https://img.shields.io/badge/Store-PicoLemon-f3cd5a?longCache=true&style=flat-rounded)](http://picolemon.com/board/PICOFABRIC)
[![Examples](https://img.shields.io/badge/Code-Examples-f3cd5a?longCache=true&style=flat-rounded)](https://github.com/picolemon/picofabric-examples)
[![Discord](https://img.shields.io/badge/@-Discord-f3cd5a?longCache=true&style=flat-rounded)](https://discord.gg/Be3yFCzyrp)

![HyperRam board overview](doc/images/csdk.png)

### Overview :hammer:
Provides an interface to the PicoFabric FPGA over SPI. Allows programming bitstream and accessing board features.

### Whats included :musical_note:
- [x] FPGA Bit Stream programming example.
- [x] Bit Stream uploading library.
- [x] Pico UF2 bootloader & source used by the PicoFabric IDE to program FPGA over USB serial.
- [x] Device CLI programmer .


### Building the example :mag:
A simple FPGA bitstream programmer example "sw/pico/projects/programing_example" is provided, this uploads a bit stream stored in flash embedded as a c array.

- Install the Pico SDK toolchain & cmake.

- Init cmake project, open a terminal and cd into the repo root which should contain a folder called sw/.
```
cd sw/pico
mkdir build
cd build
cmake ..
```

- Navigate to the programing_example example inside the build folder.
```
cd projects/programing_example
make
```

- Upload the UF2 image to the Pico and the LED should blink on the FPGA board.

- To change the bitstream use the headerembed.py tool listed below.


### Usage in existing project :mag:
A simpler way to use libfabric is to drop the source into your existing project.
- Copy libfabric.c & libfabric.h from the "sw/pico/projects/libfabric/" folder into your project.

- Build a bitstream using the PicoFabric IDE or other software then to embed as a header. This encodes the file as a c include and declares an array named bitstream as an uint8 * type.
```
$ python tools/headerembed.py data/blinky.bit bitstream.bit.h bitstream
```

```
#include <stdio.h>
#include "libfabric.h"
#include "bitstream.bit.h"

int main()
{
    struct FPGA_config_t config;
    fpga_init_config( &config, BOARD_FABRIC12k );
    fpga_program_device( &config, bitstream, bitstream_size );
}
```

- Add to CMakeLists.txt eg.
```
target_sources(myapp PRIVATE
        libfabric.c
		main.c
        )	

target_link_libraries(myapp libfabric hardware_clocks hardware_spi)        
```


Alternatively see "sw/pico/projects/program_example" on how to link the libfabric library and usage.


### Using the CLI programmer :mag:

- Install the "sw/programmer/fabric_bootloader.uf2" UF2 image to the Pico micro controller using BOOTSEL mode.
- Run ```python sw/programmer/program.py bitstream.bit``` to program the device.


### Related libraries :mag:
- The [PicoFabric MicroPython library](https://github.com/picolemon/picofabric-micropython) is also available.


### Support :zap:
- Drop by the [discord](https://discord.gg/Be3yFCzyrp)
- Email help@picolemon.com

### License :penguin:
 
The MIT License (MIT)

Copyright (c) 2023 picoLemon

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
