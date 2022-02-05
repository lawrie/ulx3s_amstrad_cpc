# Amstrad CPC on the Ulx3s Ecp5 FPGA board

## Introduction

This is an emulation of the Amstrad CPC 664, with one disk drive.

The rom is 48KB and comprises  OS664 + BASIC664 + AMSDOS.

It uses a PS/2 keyboard connected to the us2 USB port.

The output is 640x480 VGA over HDMI.

Audio is supported (mono only).

It supports an OSD for for loading floppy disk images with a .dsk extension.

It only works on an 85F because of the amount of BRAM it uses.

## Installation

You need to run micropython on the ESP32 and copy the files in the esp32 directory to it.

To start the OSD, you need to do "import osd" in the micropython repl.

Once you have programmed the bitstream, you should see the "Amstrad 64K Microcomputer (v2)" message and a BASIC 1.1 prompt.

To load a .dsk image you press all four direction buttons to start the OSD, select the required file, and press the right button.

You can then use the "cat" command to list the disk's contents and the load command to load a program from the disk.

## Limitations

BRAM is used for video memory so it is fixed at address 0xC000.

The CPU runs at 4MHz but it slightly faster than a real Amstrad CPC as the CPU and video generator do not share memory.

Only one disk is supported and it is read-only.

No extra roms are supported.

There is no joystick support.

Tape loading is not supported.
