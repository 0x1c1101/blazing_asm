# Introduction

This guide outlines a high-performance assembler framework aimed at explicit machine instruction-level byte manipulation. The assembler allows you to construct and modify shellcode sequences efficiently through direct handling of CPU registers, memory locations, and control flow instructions.

Unlike usual assemblers generating text-based assembly code, this system is low-level, generating raw bytecode with minimal overhead. This enables execution at speeds of lightning and instruction encoding with detailed control, optimally suited for performance-critical applications such as runtime code generation, exploit writing, or embedded systems programming.

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

# Labels

To define a label, there's a `Label()` macro that casts the integer to `size_t` since 64-bit immediate operands are mostly prohibited. Currently, labels only support control flow instructions (`JCC`, `JMP`, `JMP32`, `CALL`, `CALL32`) to simplify the development of such shellcodes. In order to place a label at a certain position, use the `BIND()` instruction. Afterwards you can pass the label variable into said instructions. Make sure to define each label uniquely for every `assemble()` operation. Connecting instructions with the specified labels is done with the help of a hash map. For example, if you have bounded 2 labels each ID will be reduced to 2 modulo (`ID % 2`). Therefore in the case of 2 total bounded labels, initializing them with `0` and `3` will lead to a collision. There won't be any errors but those two labels will point to the last bounded label offset.

```cpp
constexpr auto loop = Label(0); // constexpr size_t loop = 0;
constexpr auto done = Label(1); // constexpr size_t done = 1;

auto shellcode = assemble(
TEST(R10, R10),
JCC(JZ, done), // jz done

BIND(loop), // .loop:

XOR(RAX, RAX),
ADD(R13, uint8_t(4)),
SUB(R10, uint8_t(1)),
TEST(R10, R10),
JCC(JNZ, loop), // jnz loop

BIND(done), // .done:

RET()
);
```

Due to the design limitations, we can't evaluate labels during emission so a slight decrease in speed at runtime is expected. This is obviously not the case with static constexpr emitter.

Also a side note: Branch operations with labels do not support short jumps that take only a byte. For the sake of simplicity, 32-bit displacement is used when measuring the distance between the label and the instruction. If you want a smaller shellcode, you can hardcode the displacement as `int8_t`.

# Static and Dynamic emitter

When `constexpr auto shellcode = assemble_static(...)` is used, assembler emits the entire code at compile-time. Which means zero runtime overhead, just like initializing a regular hardcoded `std::array<uint8_t>`.

On the other hand, using `auto shellcode = assemble(...)` still does the necessary work at compile-time. However it enables generating desired variables at runtime while maintaining the performance. Some operations are inlined because they can't be determined until the execution. It is still quite fast compared to text-based assemblers.

# Recommendations

Enabling [precompiled headers](https://en.wikipedia.org/wiki/Precompiled_header) is highly recommended to reduce compilation time as well. The compiler simply converts this header file into a compiled shared object, allowing you to use it without having to compile it each time.



