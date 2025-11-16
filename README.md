# MOS 6502 Emulator

A headless emulator of the MOS Technology 6502 CPU, written in standard C99.

![License](https://img.shields.io/badge/License-MIT-blue)
![Windows](https://github.com/daniilsjb/mos6502.c/actions/workflows/windows.yml/badge.svg)
![macOS](https://github.com/daniilsjb/mos6502.c/actions/workflows/macos.yml/badge.svg)
![Linux](https://github.com/daniilsjb/mos6502.c/actions/workflows/linux.yml/badge.svg)

## Features

* Implements all 151 of the [official opcodes](https://www.masswerk.at/6502/6502_instruction_set.html) for the MOS 6502 CPU.
* Provides [cycle-accuracy](./tests/suite_opcodes.c) by emulating every single memory read and write.
* Optionally supports [binary-coded decimal](http://www.6502.org/tutorials/decimal_mode.html) arithmetic for platforms that allow it.
* Correctness extensively verified through tens of thousands of [automated tests](./tests).
* Standalone and cross-platform library relying only on type definitions of C99.
* Easily pluggable into any project - simply copy two files, and you're done!

## Background

This code was originally written for a NES emulator, starting in 2021. However, given the complexity and reusability of
a CPU emulator, it made sense to treat it as a standalone library with its own repository. I plan on using this code as
part of emulators for other systems, so feel free to use it in your own projects as well!

## Usage

Simply copy `mos6502.h` and `mos6502.c` to any location in your project. You may either compile the source code along
with the rest of the project, or compile it into a static or dynamic library linked against your target. If necessary,
adjust include paths in the implementation file. Then simply include the header in your own code.

For an example of using the library's API, refer to the Usage section of the [`mos6502.h`](./mos6502.h) header. You may
also find complete samples of running 6502 programs under [tests](./tests). However, as a very brief overview, the
typical flow of emulating a 6502-based system will consist of roughly the same steps:

1. Define the structure of the system's state (i.e., its components: CPU, RAM, I/O, etc.)
2. Define the read and write callbacks for the CPU to interact with the system's devices.
3. Create and initialize a CPU emulator instance using the `mos6502_create()` function.
4. Load the contents of your program into memory accessible to the CPU via callbacks.
5. Trigger the RESET interrupt via `mos6502_reset()` to prepare for program execution.
6. Call `mos6502_step()` in a loop, running one instruction at a time, until you're done.

## Testing

To run the tests for this project, you will need the following to be installed and available on your system:

* C11 (e.g., MSVC, GCC, or Clang)
* CMake 3.30

After cloning the repository, testing the emulator is as simple as running standard CMake commands:

1. `mkdir build`
2. `cmake -S . -B ./build -DCMAKE_BUILD_TYPE=Release`
3. `cmake --build ./build --config Release`
4. `cd ./build && ctest -c Release --verbose`

To disable compiler optimizations during test execution, use `Debug` instead of `Release`.

## References

I deliberately avoided looking at other implementations of the 6502 to make the process of writing (and debugging) the
emulator more engaging. Instead, I relied primarily on various online resources, including:

- [Build a 65c02-based computer from scratch](https://www.youtube.com/playlist?list=PLowKtXNTBypFbtuVMUVXNR0z1mu7dp7eH) - a YouTube playlist by Ben Eater. 
- [6502 Instruction Set](https://www.masswerk.at/6502/6502_instruction_set.html) - an online reference of the 6502 opcodes by mass:werk.
- [Easy 6502](https://skilldrick.github.io/easy6502/index.html) - a tiny e-book on the 6502 assembly language by Nick Morgan.
- [2A03 CPU Wiki](https://www.nesdev.org/wiki/CPU) - technical findings maintained by the NesDev community.
- [6502_cpu.txt](https://www.nesdev.org/6502_cpu.txt) - a cycle-by-cycle breakdown by John West and Marko Mäkelä.
- [R650X Datasheets](http://6502.org/documents/datasheets/rockwell/) - hardware documentation of Rockwell microprocessors.

## License

MIT
