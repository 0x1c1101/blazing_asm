# Blazing ASM
<img src="https://github.com/user-attachments/assets/562c7058-7e2e-40ff-be93-7a3b57d940cb" width="100" />


[![Discord](https://img.shields.io/badge/chat-on%20Discord-green.svg)](https://discord.gg/GdYanwSCwm)
[![License: GPL-3.0](https://img.shields.io/badge/License-GPL3-blue.svg)](LICENSE)


A simple and lightweight assembler library that supports the majority of significant instructions from the Intel x86-64 architecture. Mainly beneficial for shellcode development. Due to its design, it can produce the machine code quite fast while allowing you to use random registers/immediates at runtime.

## Requirements

- C++20 or above
- STL is not required (Compatible with kernel driver projects)

## Why not use AsmJit?

If you need to generate x86-64 machine code with minimal overhead, `BlazingASM` is a powerful alternative to dynamic assemblers like AsmJit. Designed for compile-time code generation, it emits raw and fixed-size machine instructions directly into static arrays, eliminating the need for runtime encoding or memory management. Single C++ header makes it really easy to integrate ~ even in kernel or embedded system projects. With a predictable layout, it's able to construct instructions at compile-time while handling dynamic operands at runtime without overhead. Particularly useful in exploit development, low-level code mutation and obfuscation, penetration testing and any scenario where shellcode is involved.

That being said, `BlazingASM` is intentionally minimal and template focused. It doesn’t aim to match AsmJit’s vast instruction coverage, runtime flexibility, or feature set. Instead, it trades breadth for simplicity and performance ~ making it a practical tool when you know exactly what code you need, and want it fast and small.

There are two different approaches you can utilize:

#### Static Mode
For a better demonstration, below is a screenshot of the comparison between the code and the disassembly (with `assemble_static()`):

![assemble_static_disassembly](https://github.com/user-attachments/assets/97f339c1-a1e7-4b66-a3f6-4e8e3e774ead)

#### Dynamic Mode
If you wish to use the dynamic code generation, below is a screenshot of the comparison between the code and the decompilation on IDA (with `assemble()`):

![assemble_decompile](https://github.com/user-attachments/assets/ffa45f56-aefe-4b4e-afec-241210ca3413)


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

All you need to do is download the `blazing_asm.hpp` header file, paste it into your project folder, and write this into your source code:

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

You can also use `basm` namespace within a scope for a simpler syntax:

```cpp
{
  using namespace basm;
  
  auto shellcode = assemble_static (
    MOV (RAX, RBX),
    MOV (RCX, RDX)
  );
}
```

## Usage

You can find the detailed explanation in the [Documentation Page](DOCUMENTATION.md).

## 🚧 Note

This project is currently in development, so if you come across any problems, please create an issue. Feel free to contribute and improve.
