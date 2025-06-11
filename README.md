# Blazing ASM
<img src="https://github.com/user-attachments/assets/562c7058-7e2e-40ff-be93-7a3b57d940cb" width="100" />


[![Discord](https://img.shields.io/badge/chat-on%20Discord-green.svg)](https://discord.gg/GdYanwSCwm)
[![License: GPL-3.0](https://img.shields.io/badge/License-GPL3-blue.svg)](LICENSE)


A simple and lightweight assembler library that supports the majority of significant instructions from the Intel x86-64 architecture. Mainly beneficial for shellcode development and runtime bytecode generation. Due to its design, it can produce the machine code quite fast while allowing you to generate and use random registers/immediates at runtime.

## Why not use AsmJit?

If you need to generate x86-64 machine code with minimal overhead and maximum control, `BlazingASM` is a powerful alternative to dynamic assemblers like AsmJit. Designed entirely for compile-time code generation, it emits raw, fixed-size machine instructions directly into static arrays, eliminating the need for runtime encoding or memory management. Its single-header design requires no dependencies, making it lightweight and easy to integrate ~ even in bare-metal or embedded environments. With predictable instruction layout, very little to zero runtime allocation, and clean, assembly-like syntax, it is particularly well-suited for shellcode generation, firmware, code mutation or any scenario where performance, binary size, and determinism matter.

That being said, `BlazingASM` is intentionally minimal and focused; it doesn’t aim to match AsmJit’s vast instruction coverage, runtime flexibility, or feature set. Instead, it trades breadth for simplicity and performance ~ making it a practical tool when you know exactly what code you need, and want it fast and small.

For a better demonstration, below is a screenshot of the comparison between the code and the decompilation (`assemble_static()` mode):

![assemble_static_disassembly](https://github.com/user-attachments/assets/97f339c1-a1e7-4b66-a3f6-4e8e3e774ead)


## Instructions Supported

| **Category**              | **Instruction**                                        | **Description**                                                    |
| ------------------------- | ------------------------------------------------------ | ------------------------------------------------------------------ |
| **Data Movement**         | `MOV`                                                  | Register/Memory/Immediate moves                                    |
|                           | `LEA`                                                  | Load effective address                                             |
|                           | `DB(...)`                                              | Define raw byte(s) (data injection)                                |
| **Arithmetic (GRP1)**     | `ADD`, `OR`, `ADC`, `SBB`, `AND`, `SUB`, `XOR`, `CMP`  | Basic arithmetic and bitwise operations                            |
|                           | `TEST`                                                 | Bitwise AND for flag setting (no result stored)                    |
| **Shifts/Rotates (GRP2)** | `ROL`, `ROR`, `RCL`, `RCR`, `SHL`, `SAL`, `SHR`, `SAR` | Support both constant and 1-bit variants (`ROL1`, `ROR1`, etc.)    |
| **Unary Ops (GRP3)**      | `NOT`, `NEG`                                           | Logical negation, two's complement                                 |
|                           | `MUL`, `IMUL`                                          | Unsigned and signed multiplication                                 |
|                           | `DIV`, `IDIV`                                          | Unsigned and signed division                                       |
| **Control Flow**          | `JCC`                                                  | Conditional jumps via `Conditions` enum                            |
|                           | `JMP`, `JMP32`                                         | Unconditional jump (near and 32-bit override)                      |
|                           | `CALL`, `CALL32`                                       | Near calls (with 32-bit override support)                          |
| **Stack Operations**      | `PUSH`, `PUSH32`, `POP`, `POP32`                       | Push/pop values (supports 64-bit and 32-bit override)              |
|                           | `POPF`, `PUSHF`                                        | Push/pop flags                                                     |
| **Fixed Instructions**    | `RET`                                                  | Return from procedure                                              |
|                           | `NOP`                                                  | No operation                                                       |
| **Planned**               |  `FPU Ops`, `INC/DEC`                                  | 🚧 Coming Soon...                                                  |

## Setting it up

All you need to do is download the `blazing_asm.hpp` header file, paste it into your project folder, and add this in your source code:

```cpp
#include "blazing_asm.hpp"

.
.
.

auto shellcode = basm::assemble_static (
  basm::MOV (basm::RAX, basm::RBX),
  basm::MOV (basm::RCX, basm::RDX)
);
```

Or you can define `basm` namespace for a simpler syntax:

```cpp
using namespace basm;

auto shellcode = assemble_static (
  MOV (RAX, RBX),
  MOV (RCX, RDX)
);
```

## Usage

You can find the detailed explanation in the [Documentation Page](DOCUMENTATION.md).

## 🚧 Note

This project is currently in development, so if you come across any problems, please create an issue. Also, feel free to contribute and improve.
