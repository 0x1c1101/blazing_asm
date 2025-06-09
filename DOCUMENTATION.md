# Introduction

This guide outlines a high-performance assembler framework aimed at explicit machine instruction-level byte manipulation. The assembler allows you to construct and modify shellcode sequences efficiently through direct handling of CPU registers, memory locations, and control flow instructions.

Unlike usual assemblers generating text-based assembly code, this system is low-level, generating raw bytecode with minimal overhead. This enables execution at speeds of lightning and instruction encoding with detailed control, optimally suited for performance-critical applications such as runtime code generation, exploit writing, or embedded systems programming.

In the following sections, you will find a detailed explanation on how to perform each of the instructions employed, what registers to use and how they compare to machine registers, and how you can tailor and integrate this assembler into your specifications.

# Operand Types

There are 3 kind of operands:

- Immediate
- Register
- Memory

## Immediate Operands

To use immediate operands, you can cast the constant depending on the size you want. For example:

```cpp
MOV(RAX, uint8_t(0x13)); // 1 Byte Immediate
MOV(AX, uint16_t(0x13)); // 2 Byte (WORD) Immediate
MOV(EAX, uint32_t(0x13371337)); // 4 Byte (DWORD) Immediate
MOV(EAX, uint64_t(0x13371337)); // 8 Byte (QWORD) Immediate
```

Some instructions might disallow the use of various sized immediate operands, compiler is going to let you know about it.

If you are going to use the immediate as a displacement, you can also specify signed types such as:

```cpp
JCC(JE, int8_t(-0x13)); // 1 Byte Signed Immediate
JCC(JE, int16_t(-0x13)); // 2 Byte (WORD) Signed Immediate
JCC(JE, int32_t(-0x13371337)); // 4 Byte (DWORD) Signed Immediate
```

## Register Operands

Registers are defined separately as enum types (`Reg64`, `Reg32`, `Reg16`, `Reg8`, `...`). And unless you want to generate random values and cast them to these enums, you can directly use register values:

```cpp
using namespace basm;

MOV(RAX, R8);
ADD(R12, uint8_t(0x13));
XOR(SIL, R9B);
```

## Memory Operands

The memory operand consists of a proxy struct that is used to specify the size of the pointer (`MEM.QWORD`, `MEM.DWORD`, `MEM.WORD`, `MEM.BYTE`). Assembly-like syntax is acquired with the help of operator overloading, improving the readability. You can set registers and displacement inside of the `[]` operator. However for the scaling, you have to use the `ScalingSize` enum instead of integers (`2`, `4`, `8`) with multiplication operator (`*`).  It is to prevent registers from losing their value since the compiler takes it as an arithmetic operation.

```cpp
using namespace basm;

MOV(RAX, MEM.QWORD[RAX]); // Base register: RAX
LEA(RAX, MEM.QWORD[R8]); // Load Effective Address to RAX
MOV(RAX, MEM.QWORD[EAX]); // Base register: EAX
MOV(AX,  MEM.WORD[RAX]); // Base register: RAX (WORD Operation)

MOV(RAX, MEM.QWORD[RAX + int8_t(0x10)]); // Base + Displacement (8 bits)
MOV(RAX, MEM.QWORD[RAX - int8_t(0x10)]); // Base - Displacement (8 bits)

MOV(RAX, MEM.QWORD[RAX + RBX]); // Base + Index * 1
MOV(RAX, MEM.QWORD[RAX + RBX + int8_t(0x10)]); // Base + Index * 1 + Displacement (8 bits)
MOV(RAX, MEM.QWORD[RAX + RBX + int32_t(0x10101010)]); // Base + Index * 1 + Displacement (32 bits)

MOV(RAX, MEM.QWORD[RAX * SCALING_W]); // Index register: RAX * 2 + 0x00 (Displacement)
MOV(RAX, MEM.QWORD[RAX * SCALING_D]); // Index register: RAX * 4 + 0x00 (Displacement)
MOV(RAX, MEM.QWORD[RAX * SCALING_Q]); // Index register: RAX * 8 + 0x00 (Displacement)

MOV(RAX, MEM.QWORD[RIP + int8_t(0x10)]); // Relative memory address
MOV(RAX, MEM.QWORD[RIP + int32_t(0x10101010)]); // Relative memory address
MOV(RAX, MEM.QWORD[EIP + int8_t(0x10)]); // Relative memory address (32-bit)

LEA(RAX, MEM.QWORD[int32_t(0x10101010)]); // Pointer to DS:0x10101010 (Data Segment)

```

Note: If you don't specify the displacement type (`int8_t` etc.), the default type will be `int` a.k.a. `int32_t` and it will take extra space (4 bytes) since the assembler cannot determine the type depending on the value.

```cpp
MOV(RAX, MEM.QWORD[RAX + 4]); // Base + Displacement (32 bits) => Not 8 bits
```

Another Note: If you are going to write 32-bit assembly, make sure to use `MEM32` variable instead of `MEM`:

```cpp
MOV(ECX, MEM32.DWORD[EAX + int8_t(4)]); // Base + Displacement (8 bits)
```

When `MEM32` is used, the assembler doesn't append the `0x67` (32-bit Register mode in a memory operand for x64 operations) prefix byte, allowing you to write 32-bit only assembly code.
