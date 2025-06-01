/*
    Copyright (c) 2025 - Present, 0x1c1101 (a.k.a. heapsoverflow) and contributors

    Permission is hereby granted, free of charge, to any person obtaining
    a copy of this software and associated documentation files (the
    "Software"), to deal in the Software without restriction, including
    without limitation the rights to use, copy, modify, merge, publish,
    distribute, sublicense, and/or sell copies of the Software, and to
    permit persons to whom the Software is furnished to do so, subject to
    the following conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
    LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
    OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
    WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

    --- Optional exception to the license ---

    As an exception, if, as a result of your compiling your source code, portions
    of this Software are embedded into a machine-executable object form of such
    source code, you may redistribute such embedded portions in such object form
    without including the above copyright and permission notices.

*/

// See https://github.com/0x1c1101/blazing_asm for updates and documentation


#pragma once

#ifndef BLAZING_ASSEMBLER_HPP
#define BLAZING_ASSEMBLER_HPP

#include <array>
#include <type_traits>

namespace blazing_asm {


#pragma region Definitions

    struct nulltype_t {

    };

    // ---- Register Enum ----


    enum ScalingSize : uint8_t { SCALING_NONE = 0b00, SCALING_W = 0b01, SCALING_D = 0b10, SCALING_Q = 0b11 };

    enum OP_EXTENSION : uint8_t {
        OP_ADD = 0b000,
        OP_OR = 0b001,
        OP_ADC = 0b010,
        OP_SBB = 0b011,
        OP_AND = 0b100,
        OP_SUB = 0b101,
        OP_XOR = 0b110,
        OP_CMP = 0b111
    };

    enum class OperandSize : uint8_t { None = 0, Byte = 1, Word = 2, DWord = 4, QWord = 8 };
    enum class OperandType : uint8_t { None = 0, Immediate, Register, Memory };

    enum Reg64 : uint8_t { RCX = 1, RDX, RBX, RSI = 6, RDI };
    enum Reg32 : uint8_t { ECX = 1, EDX, EBX, ESI = 6, EDI };
    enum Reg16 : uint8_t { CX = 1, DX, BX, SP, BP, SI, DI };
    enum Reg8 : uint8_t { CL = 1, DL, BL };

    enum Reg8H : uint8_t { AH = 4, CH, DH, BH }; // REX disabled
    enum Reg8L : uint8_t { SPL = 4, BPL, SIL, DIL }; // REX enabled

    enum RegX  : uint8_t {  R8,  R9,  R10,  R11,  R14 = 6,  R15 }; // Extended QWORD Registers
    enum RegXD : uint8_t { R8D, R9D, R10D, R11D, R14D = 6, R15D }; // Extended DWORD Registers
    enum RegXW : uint8_t { R8W, R9W, R10W, R11W, R12W, R13W, R14W, R15W }; // Extended WORD Registers
    enum RegXB : uint8_t { R8B, R9B, R10B, R11B, R12B, R13B, R14B, R15B }; // Extended BYTE Registers


    // These registers require SIB byte in memory operands even without the index register
    enum RegRSP : uint8_t { RSP = 4 };
    enum RegESP : uint8_t { ESP = 4 };
    enum RegR12 : uint8_t { R12 = 4 };
    enum RegR12D : uint8_t { R12D = 4 };

    // These registers can't be a base register with no displacement in memory operands
    enum RegRBP : uint8_t { RBP = 5 };
    enum RegEBP : uint8_t { EBP = 5 };
    enum RegR13 : uint8_t { R13 = 5 };
    enum RegR13D : uint8_t { R13D = 5 };

    // For relative offset
    enum RegRIP : uint8_t { RIP };
    enum RegEIP : uint8_t { EIP };

    // For special opcode optimization
    enum RegRAX : uint8_t { RAX = 0 };
    enum RegEAX : uint8_t { EAX = 0 };
    enum RegAX : uint8_t { AX = 0 };
    enum RegAL : uint8_t { AL = 0 };

    template <typename T, template <typename...> class Template>
    struct is_specialization_of : std::false_type {};

    template <template <typename...> class Template, typename... Args>
    struct is_specialization_of<Template<Args...>, Template> : std::true_type {};


    // R8-R15 registers
    template<typename T>
    constexpr bool is_reg_r() {
        if constexpr (std::is_same_v<T, RegX> || std::is_same_v<T, RegXD> ||
            std::is_same_v<T, RegR12> || std::is_same_v<T, RegR13> || 
            std::is_same_v<T, RegR12D> || std::is_same_v<T, RegR13D> || 
            std::is_same_v<T, RegXW> || std::is_same_v<T, RegXB>)
            return true;
        return false;
    }

    
    template<typename T>
    constexpr bool is_register() {
        if constexpr (
            std::is_same_v<T, Reg64> ||
            std::is_same_v<T, Reg32> ||
            std::is_same_v<T, Reg16> ||
            std::is_same_v<T, Reg8> ||
            std::is_same_v<T, Reg8L> ||
            std::is_same_v<T, Reg8H> ||
            std::is_same_v<T, RegX> ||
            std::is_same_v<T, RegXD> ||
            std::is_same_v<T, RegXW> ||
            std::is_same_v<T, RegXB> ||
            std::is_same_v<T, RegRSP> ||
            std::is_same_v<T, RegESP> ||
            std::is_same_v<T, RegR12> ||
            std::is_same_v<T, RegR12D> ||
            std::is_same_v<T, RegRBP> ||
            std::is_same_v<T, RegEBP> ||
            std::is_same_v<T, RegR13> ||
            std::is_same_v<T, RegR13D> ||
            std::is_same_v<T, RegRAX> ||
            std::is_same_v<T, RegEAX> ||
            std::is_same_v<T, RegAX> ||
            std::is_same_v<T, RegAL> ||
            std::is_same_v<T, RegRIP> ||
            std::is_same_v<T, RegEIP>
            )
            return true;
        return false;
    }

    template<typename T>
    constexpr bool is_reg64() {
        if constexpr (is_register<T>() && get_op_size<T>() == OperandSize::QWord)
            return true;
        return false;
    }

    template<typename T>
    constexpr bool is_reg32() {
        if constexpr (is_register<T>() && get_op_size<T>() == OperandSize::DWord)
            return true;
        return false;
    }

    template<typename T>
    constexpr bool is_imm() {
        if constexpr (
            std::is_same_v<T, uint64_t> ||
            std::is_same_v<T, int64_t> ||
            std::is_same_v<T, uint32_t> ||
            std::is_same_v<T, int32_t> ||
            std::is_same_v<T, uint16_t> ||
            std::is_same_v<T, int16_t> ||
            std::is_same_v<T, uint8_t> ||
            std::is_same_v<T, int8_t>)
            return true;
        return false;
    }

    template<typename T>
    constexpr bool is_accumulator() {
        if constexpr (
            std::is_same_v<T, RegRAX> ||
            std::is_same_v<T, RegEAX> ||
            std::is_same_v<T, RegAX> ||
            std::is_same_v<T, RegAL>)
            return true;
        return false;
    }

    template<typename B, typename I>
    static constexpr bool require_sib() {
        if constexpr (
            std::is_same_v<B, RegRSP> ||
            std::is_same_v<B, RegESP> ||
            std::is_same_v<B, RegR12> ||
            std::is_same_v<B, RegR12D>)
            return true;
        if constexpr (!std::is_same_v<I, nulltype_t>)
            return true;

        if constexpr (std::is_same_v<B, nulltype_t>)
            return true;

        return false;
    }
    template<typename B, typename I, typename D>
    static constexpr bool require_disp() {
        if constexpr (
            std::is_same_v<B, RegRIP> ||
            std::is_same_v<B, RegEIP> ||
            std::is_same_v<B, RegRBP> ||
            std::is_same_v<B, RegEBP> ||
            std::is_same_v<B, RegR13> ||
            std::is_same_v<B, RegR13D>)
            return true;

        if constexpr (!std::is_same_v<D, nulltype_t>)
            return true;

        if constexpr (std::is_same_v<B, nulltype_t>)
            return true;

        return false;
    }


    template <typename OpSizeT = nulltype_t, typename BaseRegT = nulltype_t, typename IndexRegT = nulltype_t, typename DispSizeT = nulltype_t>
    struct Memory {
        static_assert(is_reg64<BaseRegT>() || is_reg32<BaseRegT>() || std::is_same_v<BaseRegT, nulltype_t>, "Invalid base register type");
        static_assert(is_reg64<IndexRegT>() || is_reg32<IndexRegT>() || std::is_same_v<IndexRegT, nulltype_t>, "Invalid index register type");


        static_assert(get_op_size<BaseRegT>() == get_op_size<IndexRegT>() || std::is_same_v<BaseRegT, nulltype_t> || std::is_same_v<IndexRegT, nulltype_t>, "Invalid base/index expression");

        static_assert(!std::is_same_v<OpSizeT, nulltype_t>, "Invalid operand size");
        static_assert((is_imm<DispSizeT>() && (get_op_size<DispSizeT>() == OperandSize::Byte || get_op_size<DispSizeT>() == OperandSize::DWord)) || std::is_same_v<DispSizeT, nulltype_t>, "Invalid displacement size");


        static_assert(!std::is_same_v<IndexRegT, RegRSP>, "RSP can't be an index register.");
        static_assert(!std::is_same_v<IndexRegT, RegESP>, "ESP can't be an index register.");
        static_assert(!std::is_same_v<IndexRegT, RegRIP>, "RIP can't be an index register.");
        static_assert(!std::is_same_v<IndexRegT, RegEIP>, "EIP can't be an index register.");
        static_assert(!((std::is_same_v<BaseRegT, RegEIP> || std::is_same_v<BaseRegT, RegRIP>) && !std::is_same_v<IndexRegT, nulltype_t>), "Can't use relative with an index register.");


        using BaseReg = BaseRegT;
        using IndexReg = IndexRegT;
        using OpSize = OpSizeT;
        using DispSize = DispSizeT;


        BaseRegT base;
        IndexRegT index;

        uint8_t scale = SCALING_NONE;
        DispSizeT disp;

       
    };

    template <typename Type>
    constexpr OperandSize get_op_size() {
        if constexpr (
            std::is_same_v<Type, RegRAX> ||
            std::is_same_v<Type, RegRIP> ||
            std::is_same_v<Type, RegRSP> ||
            std::is_same_v<Type, RegRBP> ||
            std::is_same_v<Type, RegR12> ||
            std::is_same_v<Type, RegR13> ||
            std::is_same_v<Type, Reg64> ||
            std::is_same_v<Type, RegX> ||
            std::is_same_v<Type, uint64_t> ||
            std::is_same_v<Type, int64_t>)
                return OperandSize::QWord;


        if constexpr (
            std::is_same_v<Type, RegEAX> ||
            std::is_same_v<Type, RegEIP> ||
            std::is_same_v<Type, RegESP> ||
            std::is_same_v<Type, RegEBP> ||
            std::is_same_v<Type, RegR12D> ||
            std::is_same_v<Type, RegR13D> ||
            std::is_same_v<Type, Reg32> ||
            std::is_same_v<Type, RegXD> ||
            std::is_same_v<Type, uint32_t> ||
            std::is_same_v<Type, int32_t>)
                return OperandSize::DWord;

        if constexpr (
            std::is_same_v<Type, RegAX> ||
            std::is_same_v<Type, Reg16> ||
            std::is_same_v<Type, RegXW> ||
            std::is_same_v<Type, uint16_t> ||
            std::is_same_v<Type, int16_t>)
                return OperandSize::Word;

        if constexpr (
            std::is_same_v<Type, RegAL> ||
            std::is_same_v<Type, Reg8L> ||
            std::is_same_v<Type, Reg8H> ||
            std::is_same_v<Type, Reg8> ||
            std::is_same_v<Type, RegXB> ||
            std::is_same_v<Type, uint8_t> ||
            std::is_same_v<Type, int8_t>)
                return OperandSize::Byte;

        if constexpr (is_specialization_of<Type, Memory>::value)
            return get_op_size<typename Type::OpSize>();
        

        return OperandSize::None;

    }

    

    template <typename Type>
    constexpr OperandType get_op_type() {
        if constexpr (is_imm<Type>())
        {
            return OperandType::Immediate;
        }
        if constexpr (is_register<Type>()) {
            return OperandType::Register;
        }
        if constexpr (is_specialization_of<Type, Memory>::value)
        {
            return OperandType::Memory;
        }
        return OperandType::None;
    }

    template <typename RegType>
    constexpr bool op_requires_rex() {

        if constexpr (is_specialization_of<RegType, Memory>::value) {
            using IndexRegT = typename RegType::IndexReg;
            using BaseRegT = typename RegType::BaseReg;

            if constexpr (is_reg_r<IndexRegT>() || is_reg_r<BaseRegT>())
                return true;
        }

        if constexpr (std::is_same_v<RegType, Reg8L> || is_reg_r<RegType>() || is_reg64<RegType>())
            return true;
        return false;
    }

    template <typename RegDest, typename RegSrc>
    constexpr uint8_t op_calculate_rex() {

        if constexpr (!is_imm<RegSrc>() || get_op_size<RegDest>() != OperandSize::QWord || !is_specialization_of<RegDest, Memory>::value)
        {
            if constexpr (!op_requires_rex<RegDest>() && !op_requires_rex<RegSrc>())
                return 0;
        }

        
        // High 8-bit registers suppress REX
        static_assert(!((op_requires_rex<RegDest>() || op_requires_rex<RegSrc>()) && (std::is_same_v<RegDest, Reg8H> || std::is_same_v<RegSrc, Reg8H>)), "Can't encode R8H registers in an instruction requiring REX prefix");
        

        uint8_t rex = 0x40; // Base REX prefix

        if constexpr (!std::is_same_v<RegSrc, nulltype_t> && get_op_size<RegDest>() == OperandSize::QWord)
            rex |= 0x08; // REX.W


        if constexpr (!is_specialization_of<RegSrc, Memory>::value) {
            // R bit (modrm.reg extension - source register)
            if constexpr (is_reg_r<RegSrc>())
                rex |= 0x04;


            if constexpr (is_specialization_of<RegDest, Memory>::value) {
                using IndexRegT = typename RegDest::IndexReg;
                using BaseRegT = typename RegDest::BaseReg;

                if constexpr (is_reg_r<BaseRegT>())
                    rex |= 0x01; // Rex.B

                if constexpr (is_reg_r<IndexRegT>())
                    rex |= 0x02; // Rex.X
            }
            else {
                if constexpr (is_reg_r<RegDest>())
                    rex |= 0x01; // B bit (modrm.rm extension - destination register)
            }
            
        }
        else {
            if constexpr (is_reg_r<RegDest>())
                rex |= 0x04;

            using IndexRegT = typename RegSrc::IndexReg;
            using BaseRegT = typename RegSrc::BaseReg;

            if constexpr (is_reg_r<BaseRegT>())
                rex |= 0x01; // Rex.B

            if constexpr (is_reg_r<IndexRegT>())
                rex |= 0x02; // Rex.X

        }

        return rex;
    }

    template <typename Type>
    struct OperandHelper {
        static constexpr OperandType type = get_op_type<Type>();
        static constexpr OperandSize size = get_op_size<Type>();
    };

    
    // -- All 32-bit conditional jump opcodes (0F 8X)

    /*enum class Condition : uint8_t {
        JE = 0x84,
        JNE = 0x85,
        JL = 0x8C,
        JLE = 0x8E,
        JG = 0x8F,
        JGE = 0x8D,
        JB = 0x82,
        JBE = 0x86,
        JA = 0x87,
        JAE = 0x83,
    };*/

#pragma endregion Definitions

    // ---- Helpers ----
    template<typename T>
    struct instr_size {
        static constexpr size_t value = T::size;
    };

    template<typename... Instrs>
    constexpr size_t total_size = (instr_size<Instrs>::value + ...);


    template<typename ValType>
    constexpr void ConstWrite(uint8_t* dest, ValType val) {
        dest[0] = val & 0xFF;
        if constexpr (std::is_same_v<ValType, uint64_t>) {
            dest[1] = (val >> 8) & 0xFF;
            dest[2] = (val >> 16) & 0xFF;
            dest[3] = (val >> 24) & 0xFF;
            dest[4] = (val >> 32) & 0xFF;
            dest[5] = (val >> 40) & 0xFF;
            dest[6] = (val >> 48) & 0xFF;
            dest[7] = (val >> 56) & 0xFF;
        }
        else if constexpr (std::is_same_v<ValType, uint32_t>) {
            dest[1] = (val >> 8) & 0xFF;
            dest[2] = (val >> 16) & 0xFF;
            dest[3] = (val >> 24) & 0xFF;

        }
        else if constexpr (std::is_same_v<ValType, uint16_t>) {
            dest[1] = (val >> 8) & 0xFF;
        }

    }

    template <typename MemoryT>
    constexpr void mem_calc_op_size(size_t &sz) {
        using BaseRegT = typename MemoryT::BaseReg;
        using IndexRegT = typename MemoryT::IndexReg;
        using DispSizeT = typename MemoryT::DispSize;

        sz++; // MODRM

        if constexpr (is_reg32<BaseRegT>() || is_reg32<IndexRegT>())
            sz++;


        if constexpr (require_disp<BaseRegT, IndexRegT, DispSizeT>())
        {
            if constexpr (std::is_same_v<BaseRegT, nulltype_t> || std::is_same_v<BaseRegT, RegRIP> || std::is_same_v<BaseRegT, RegEIP>)
                sz += 4;
            else if constexpr (std::is_same_v<DispSizeT, nulltype_t>)
                sz++;
            else
                sz += size_t(get_op_size<DispSizeT>());
            
        }

        if constexpr (require_sib<BaseRegT, IndexRegT>())
            sz++;

    }

    template <typename BaseReg, typename IndexReg, typename DispSize>
    constexpr uint8_t mem_get_mod () {
        

        if constexpr (!require_disp<BaseReg, IndexReg, DispSize>())
            return 0b00; // None

        if constexpr (std::is_same_v<BaseReg, nulltype_t> || std::is_same_v<BaseReg, RegRIP> || std::is_same_v<BaseReg, RegEIP>)
            return 0b10; // DWord

        if constexpr (std::is_same_v<DispSize, nulltype_t> || get_op_size<DispSize>() == OperandSize::Byte)
            return 0b01; // Byte

        return 0b10; // DWord

    }

    template <typename MemoryT, typename RegT, typename isSrcMem = nullptr_t>
    constexpr void mem_handle_op (uint8_t* out, size_t &index, const MemoryT &mem, const RegT &reg) {
        using IndexRegT = typename MemoryT::IndexReg;
        using BaseRegT = typename MemoryT::BaseReg;
        using DispSizeT = typename MemoryT::DispSize;


        constexpr uint8_t mod = mem_get_mod<BaseRegT, IndexRegT, DispSizeT>();


        if constexpr (std::is_same_v<BaseRegT, RegRIP> || std::is_same_v<BaseRegT, RegEIP>) {
            out[index++] = 0x5; // Relative
        }
        else if constexpr (require_sib<BaseRegT, IndexRegT>()) {
            if constexpr (std::is_same_v<BaseRegT, nulltype_t>) {


                if constexpr (std::is_same_v<RegT, nulltype_t>)
                    out[index++] = 0x4;
                else
                    out[index++] = ((reg & 0x7) << 3) | (0x4); // (0b00 << 6) | ((reg & 0x7) << 3) | (0x4)

                if constexpr (std::is_same_v<IndexRegT, nulltype_t>) {
                    
                    out[index++] = 0x25; // Data segment
                }
                else {
                    out[index++] = ((mem.scale & 0b11) << 6) | ((mem.index & 0x7) << 3) | (0b101);
                }

            }
            else {

                if constexpr (std::is_same_v<RegT, nulltype_t>)
                    out[index++] = (mod << 6) | 0x4;
                else
                    out[index++] = (mod << 6) | ((reg & 0x7) << 3) | (0x4);

                if constexpr (std::is_same_v<IndexRegT, nulltype_t>)
                {
                    if constexpr (
                        std::is_same_v<BaseRegT, RegRSP> ||
                        std::is_same_v<BaseRegT, RegESP> ||
                        std::is_same_v<BaseRegT, RegR12> ||
                        std::is_same_v<BaseRegT, RegR12D>)

                        out[index++] = 0x24;
                    else
                        out[index++] = ((mem.base & 0x7));
                }
                else
                    out[index++] = ((mem.scale & 0b11) << 6) | ((mem.index & 0x7) << 3) | ((mem.base & 0x7));

            }
        }
        else if constexpr (!std::is_same_v<isSrcMem, nulltype_t>)
        {
            if constexpr (std::is_same_v<RegT, nulltype_t>)
                out[index++] = (mod << 6) | (mem.base & 0x7);
            else
                out[index++] = (mod << 6) | ((reg & 0x7) << 3) | (mem.base & 0x7);
            
        }
        else
        {
            if constexpr (std::is_same_v<RegT, nulltype_t>)
                out[index++] = (mod << 6) | ((mem.base & 0x7) << 3);
            else
                out[index++] = (mod << 6) | ((mem.base & 0x7) << 3) | (reg & 0x7);
        }
        


        if constexpr (mod == 0b01) {
            if constexpr (std::is_same_v<DispSizeT, nulltype_t>)
                ConstWrite<uint8_t>(out + index, static_cast<uint8_t>(0));
            else
                ConstWrite<uint8_t>(out + index, static_cast<uint8_t>(mem.disp));

            index++;
        }
        else if constexpr (mod == 0b10)
        {

            if constexpr (std::is_same_v<DispSizeT, nulltype_t>)
                ConstWrite<uint32_t>(out + index, static_cast<uint32_t>(0));
            else
                ConstWrite<uint32_t>(out + index, static_cast<uint32_t>(mem.disp));

            index += 4;

        }


    }


    template <typename FirstType, typename SecondType>
    constexpr void handle_prefix(uint8_t* out, size_t& index, const FirstType& op1_val, const SecondType& op2_val) {
        constexpr OperandHelper<FirstType> op1;
        constexpr OperandHelper<SecondType> op2;

        if constexpr (op1.type == OperandType::Memory) {


            using IndexRegT = typename FirstType::IndexReg;
            using BaseRegT = typename FirstType::BaseReg;

            if constexpr (is_reg32<BaseRegT>() || is_reg32<IndexRegT>())
                out[index++] = 0x67;
        }
        else if constexpr (op2.type == OperandType::Memory) {


            using IndexRegT = typename SecondType::IndexReg;
            using BaseRegT = typename SecondType::BaseReg;

            if constexpr (is_reg32<BaseRegT>() || is_reg32<IndexRegT>())
                out[index++] = 0x67;
        }

        if constexpr (op1.size == OperandSize::Word || (!is_imm<SecondType>() && op2.size == OperandSize::Word))
            out[index++] = 0x66;

        constexpr uint8_t rex = op_calculate_rex<FirstType, SecondType>();

        if constexpr (rex != 0)
            out[index++] = rex;

        


    }

#pragma region Instructions

    // ---- MOV ----
    template <typename FirstType, typename SecondType>
    struct MovInstr {
        static constexpr OperandHelper<FirstType> op1;
        static constexpr OperandHelper<SecondType> op2;

        

        FirstType op1_val;
        SecondType op2_val;


        static constexpr size_t calc_array_size() {

            

            static_assert(!std::is_same_v<FirstType, RegRIP> && !std::is_same_v<FirstType, RegEIP> && !std::is_same_v<SecondType, RegRIP> && !std::is_same_v<SecondType, RegEIP>, "MOV: Can't use RIP/EIP");
            static_assert(op1.type != OperandType::Immediate, "MOV: First operand can't be an Immediate");
            static_assert(op1.type != OperandType::Memory || op2.type != OperandType::Memory, "MOV: Memory to memory is illegal");
            static_assert(op1.type != OperandType::None, "MOV: Unknown first operand type");
            static_assert(op2.type != OperandType::None, "MOV: Unknown second operand type");

            static_assert(op1.size != OperandSize::None, "MOV: Unknown first operand size");
            static_assert(op2.size != OperandSize::None, "MOV: Unknown second operand size");

            static_assert(!(op2.type != OperandType::Immediate && op1.size != op2.size), "MOV: Both operands have to be the same size");
            static_assert(!(op2.type == OperandType::Immediate && op1.size < op2.size), "MOV: Immediate is higher than the register");
            static_assert(!(op1.type == OperandType::Memory && op2.type == OperandType::Immediate && op2.size == OperandSize::QWord), "MOV: Can't move 64 bit immediate to a memory operand.");
            static_assert(op1.size == OperandSize::QWord && op2.size == OperandSize::DWord || !(op1.type == OperandType::Memory && op2.type == OperandType::Immediate && op1.size != op2.size), "MOV: Memory operand size mismatch.");

            // mov reg, imm
            // mov reg, reg
            // mov reg, mem
            // mov mem, imm
            // mov mem, reg

            size_t sz = 1;

            if constexpr (op_calculate_rex<FirstType, SecondType>() != 0)
                sz++;

            if constexpr (op1.size == OperandSize::Word || (!is_imm<SecondType>() && op2.size == OperandSize::Word))
                sz++;

            if constexpr (op2.type == OperandType::Immediate) {
                if constexpr (op1.type == OperandType::Register) {
                    if constexpr (op1.size == OperandSize::QWord && op2.size < OperandSize::QWord)
                        sz += size_t(OperandSize::DWord) + 1;
                    else
                        sz += size_t(op1.size);
                }
                else
                    sz += size_t(op2.size);
            }
            else if constexpr (op1.type == OperandType::Register && op2.type == OperandType::Register)
                sz++;

            if constexpr (op1.type == OperandType::Memory) 
                mem_calc_op_size<FirstType>(sz);
            else if constexpr (op2.type == OperandType::Memory)
                mem_calc_op_size<SecondType>(sz);

            return sz;
        }

        static constexpr size_t size = calc_array_size();


        constexpr std::array<uint8_t, size> encode_header() const {
            std::array<uint8_t, size> out = {};
            size_t index = 0;

            handle_prefix<FirstType, SecondType>(out.data(), index, op1_val, op2_val);

            if constexpr (op1.type == OperandType::Register) {

                

                if constexpr (op2.type == OperandType::Immediate) {
                    if constexpr (op1.size == OperandSize::QWord && op2.size < OperandSize::QWord)
                        out[index++] = 0xC7;

                    out[index] = 0xB0;

                    if constexpr (op1.size != OperandSize::Byte)
                        out[index] += 0x8;
                    
                    if constexpr (op1.size == OperandSize::QWord && op2.size < OperandSize::QWord)
                        out[index] += 0x8;

                    out[index] |= (op1_val & 7);
                    index++;
                    
                    if constexpr (op2.size == OperandSize::Byte)
                        ConstWrite<uint8_t>(out.data() + index, static_cast<uint8_t>(op2_val));
                    else if constexpr (op2.size == OperandSize::Word)
                        ConstWrite<uint16_t>(out.data() + index, static_cast<uint16_t>(op2_val));
                    else if constexpr (op2.size == OperandSize::DWord || (op1.size == OperandSize::QWord && op2.size < OperandSize::QWord))
                        ConstWrite<uint32_t>(out.data() + index, static_cast<uint32_t>(op2_val));
                    else if constexpr (op2.size == OperandSize::QWord)
                        ConstWrite<uint64_t>(out.data() + index, static_cast<uint64_t>(op2_val));
                    

                }
                else if constexpr (op2.type == OperandType::Register) {

                    out[index] = 0x88;

                    if constexpr (op1.size != OperandSize::Byte)
                        out[index] += 0x1;

                    index++;

                    out[index++] = (0b11 << 6) | (op2_val & 0x7) << 3 | (op1_val & 0x7);
                    

                }
                else if constexpr (op2.type == OperandType::Memory) {
                    out[index] = 0x8A;

                    if constexpr (op1.size != OperandSize::Byte)
                        out[index] += 0x1;
                    index++;

                    mem_handle_op<SecondType, FirstType, std::true_type>(out.data(), index, op2_val, op1_val);
                }
            }
            else if constexpr (op1.type == OperandType::Memory) {

                if constexpr (op2.type == OperandType::Immediate) {

                    out[index] = 0xC6;

                    if constexpr (op1.size != OperandSize::Byte)
                        out[index] += 0x1;

                    index++;

                    mem_handle_op<FirstType, nulltype_t>(out.data(), index, op1_val, {});

                    if constexpr (op2.size == OperandSize::Byte)
                        ConstWrite<uint8_t>(out.data() + index, static_cast<uint8_t>(op2_val));
                    else if constexpr (op2.size == OperandSize::Word)
                        ConstWrite<uint16_t>(out.data() + index, static_cast<uint16_t>(op2_val));
                    else if constexpr (op2.size == OperandSize::DWord)
                        ConstWrite<uint32_t>(out.data() + index, static_cast<uint32_t>(op2_val));

                }
                else {
                    out[index] = 0x88;

                    if constexpr (op1.size != OperandSize::Byte)
                        out[index] += 0x1;
                
                    index++;

                    mem_handle_op<FirstType, SecondType>(out.data(), index, op1_val, op2_val);
                }
                
            }
            return out;
        }

        inline std::array<uint8_t, size> encode() const {
            auto header = encode_header();
            return header;
        }


        constexpr std::array<uint8_t, size> encode_constexpr() const {
            auto header = encode_header();
            return header;
        }
    };

    // ---- LEA ----
    template <typename FirstType, typename SecondType>
    struct LeaInstr {
        static constexpr OperandHelper<FirstType> op1;
        static constexpr OperandHelper<SecondType> op2;


        FirstType op1_val;
        SecondType op2_val;


        static constexpr size_t calc_array_size() {
            static_assert(!std::is_same_v<FirstType, RegRIP> && !std::is_same_v<FirstType, RegEIP> && !std::is_same_v<SecondType, RegRIP> && !std::is_same_v<SecondType, RegEIP>, "LEA: Can't use RIP/EIP");

            static_assert(op1.type == OperandType::Register && op2.type == OperandType::Memory, "LEA: Operand structure has to be in REG, MEM format");

            static_assert(op1.type != OperandType::None, "LEA: Unknown first operand type");
            static_assert(op2.type != OperandType::None, "LEA: Unknown second operand type");

            static_assert(op1.size != OperandSize::None, "LEA: Unknown first operand size");
            static_assert(op2.size != OperandSize::None, "LEA: Unknown second operand size");


            static_assert(op1.size == op2.size, "LEA: Both operands have to be the same size");
            static_assert((op1.size == OperandSize::DWord || op1.size == OperandSize::QWord) && (op2.size == OperandSize::DWord || op2.size == OperandSize::QWord), "LEA: Invalid operand size");

            // lea reg, mem


            size_t sz = 1;

            if constexpr (op_calculate_rex<FirstType, SecondType>() != 0)
                sz++;

            mem_calc_op_size<SecondType>(sz);

            return sz;
        }

        static constexpr size_t size = calc_array_size();


        constexpr std::array<uint8_t, size> encode_header() const {
            std::array<uint8_t, size> out = {};
            size_t index = 0;

            handle_prefix<FirstType, SecondType>(out.data(), index, op1_val, op2_val);

            out[index++] = 0x8D;


            mem_handle_op<SecondType, FirstType, std::true_type>(out.data(), index, op2_val, op1_val);

            return out;
        }

        inline std::array<uint8_t, size> encode() const {
            auto header = encode_header();
            return header;
        }


        constexpr std::array<uint8_t, size> encode_constexpr() const {
            auto header = encode_header();
            return header;
        }
    };


    // ---- Arithmetic/Logical Instructions ----
    template <typename FirstType, typename SecondType>
    struct ArithmeticInstr {
        static constexpr OperandHelper<FirstType> op1;
        static constexpr OperandHelper<SecondType> op2;

        FirstType op1_val;
        SecondType op2_val;

        uint8_t op_ext;

        static constexpr size_t calc_array_size() {

            static_assert(!std::is_same_v<FirstType, RegRIP> && !std::is_same_v<FirstType, RegEIP> && !std::is_same_v<SecondType, RegRIP> && !std::is_same_v<SecondType, RegEIP>, "ARITH: Can't use RIP/EIP");
            static_assert(op1.type != OperandType::Immediate, "ARITH: First operand can't be an Immediate");
            static_assert(op1.type != OperandType::Memory || op2.type != OperandType::Memory, "ARITH: Memory to memory is illegal");
            static_assert(op1.type != OperandType::None, "ARITH: Unknown first operand type");
            static_assert(op2.type != OperandType::None, "ARITH: Unknown second operand type");

            static_assert(op1.size != OperandSize::None, "ARITH: Unknown first operand size");
            static_assert(op2.size != OperandSize::None, "ARITH: Unknown second operand size");

            static_assert(!(op2.type != OperandType::Immediate && op1.size != op2.size), "ARITH: Both operands have to be the same size");
            static_assert(!(op2.type == OperandType::Immediate && op1.size < op2.size), "ARITH: Immediate is higher than the register");
            static_assert(!(op2.type == OperandType::Immediate && op2.size == OperandSize::QWord), "ARITH: Can't use 64 bit immediate.");
            //static_assert(op1.size == OperandSize::QWord && op2.size == OperandSize::DWord || !(op1.type == OperandType::Memory && op2.type == OperandType::Immediate && op1.size != op2.size), "ARITH: Memory operand size mismatch.");


            size_t sz = 1;

            if constexpr (op_calculate_rex<FirstType, SecondType>() != 0)
                sz++;

            if constexpr (op1.size == OperandSize::Word || (!is_imm<SecondType>() && op2.size == OperandSize::Word))
                sz++;

            if constexpr (op2.type == OperandType::Immediate)
            {
                if constexpr (op1.type == OperandType::Register)
                {
                    sz++;

                    if constexpr (is_accumulator<FirstType>() && op1.size == OperandSize::Byte && op2.size == OperandSize::Byte)
                        sz--;
                    else if constexpr (is_accumulator<FirstType>()) {
                        if constexpr (op2.size == OperandSize::Word || op2.size == OperandSize::DWord)
                            sz--;
                    }

                }

                if constexpr (op1.size != OperandSize::Word && op2.size == OperandSize::Word)
                    sz += 4;
                else
                    sz += size_t(op2.size);
            }
            
            else if constexpr (op1.type == OperandType::Register && op2.type == OperandType::Register)
                sz++;

            if constexpr (op1.type == OperandType::Memory)
                mem_calc_op_size<FirstType>(sz);
            else if constexpr (op2.type == OperandType::Memory)
                mem_calc_op_size<SecondType>(sz);


            return sz;
        }

        static constexpr size_t size = calc_array_size();


        constexpr std::array<uint8_t, size> encode_header() const {
            std::array<uint8_t, size> out = {};
            size_t index = 0;

            handle_prefix<FirstType, SecondType>(out.data(), index, op1_val, op2_val);

            if constexpr (op1.type == OperandType::Register) {

                if constexpr (op2.type == OperandType::Immediate) {

                    if constexpr (is_accumulator<FirstType>() && op1.size == OperandSize::Byte && op2.size == OperandSize::Byte) {
                        out[index++] = 0x04 | (op_ext << 3);
                    }
                    else if constexpr (is_accumulator<FirstType>() && op2.size != OperandSize::Byte) {
                        if constexpr (op2.size == OperandSize::Word || op2.size == OperandSize::DWord)
                            out[index++] = 0x05 | (op_ext << 3);
                    }
                    else {
                        out[index] = 0x80;

                        if constexpr (op1.size != OperandSize::Byte)
                        {
                            out[index] += 0x1;
                            if constexpr (op2.size == OperandSize::Byte)
                                out[index] += 0x2;
                        }

                        index++;
                        out[index++] = 0xC0 | (op_ext << 3) | (op1_val & 0x7);
                    }


                    if constexpr (op2.size == OperandSize::Byte)
                        ConstWrite<uint8_t>(out.data() + index, static_cast<uint8_t>(op2_val));
                    else if constexpr (op2.size == OperandSize::DWord || (op1.size != OperandSize::Word && op2.size == OperandSize::Word))
                        ConstWrite<uint32_t>(out.data() + index, static_cast<uint32_t>(op2_val));
                    else if constexpr (op2.size == OperandSize::Word)
                        ConstWrite<uint16_t>(out.data() + index, static_cast<uint16_t>(op2_val));
                    
                }
                else if constexpr (op2.type == OperandType::Register) {

                    out[index] = 0x00 | (op_ext << 3);

                    if constexpr (op1.size != OperandSize::Byte)
                        out[index] += 0x1;

                    index++;

                    out[index++] = (0b11 << 6) | (op2_val & 0x7) << 3 | (op1_val & 0x7);


                }
                else if constexpr (op2.type == OperandType::Memory) {
                    out[index] = 0x02 | (op_ext << 3);

                    if constexpr (op1.size != OperandSize::Byte)
                        out[index] += 0x1;
                    index++;

                    mem_handle_op<SecondType, FirstType, std::true_type>(out.data(), index, op2_val, op1_val);
                }
            }
            else if constexpr (op1.type == OperandType::Memory) {

                if constexpr (op2.type == OperandType::Immediate) {

                    out[index] = 0x80;

                    if constexpr (op1.size != OperandSize::Byte)
                    {
                        out[index] += 0x1;
                        if constexpr (op2.size == OperandSize::Byte)
                            out[index] += 0x2;
                    }

                    

                    index++;

                    mem_handle_op<FirstType, uint8_t>(out.data(), index, op1_val, op_ext);

                    if constexpr (op2.size == OperandSize::Byte)
                        ConstWrite<uint8_t>(out.data() + index, static_cast<uint8_t>(op2_val));
                    else if constexpr (op2.size == OperandSize::DWord || (op1.size != OperandSize::Word && op2.size == OperandSize::Word))
                        ConstWrite<uint32_t>(out.data() + index, static_cast<uint32_t>(op2_val));
                    else if constexpr (op2.size == OperandSize::Word)
                        ConstWrite<uint16_t>(out.data() + index, static_cast<uint16_t>(op2_val));

                }
                else {
                    out[index] = 0x00 | (op_ext << 3);

                    if constexpr (op1.size != OperandSize::Byte)
                        out[index] += 0x1;

                    index++;

                    mem_handle_op<FirstType, SecondType>(out.data(), index, op1_val, op2_val);
                }

            }
            return out;
        }

        inline std::array<uint8_t, size> encode() const {
            auto header = encode_header();
            return header;
        }


        constexpr std::array<uint8_t, size> encode_constexpr() const {
            auto header = encode_header();
            return header;
        }
    };
    
    
    // ---- Fixed Size Instructions ----
    enum class FixedInstrType : uint8_t {
        RET = 0,
        NOP,
        PUSHF,
        POPF
    };
    struct FixedSizeInstr {
        static constexpr std::array<uint8_t, 4> fixed_instr_table {
            0xC3,
            0x90,
            0x9C,
            0x9D
        };
        FixedInstrType index;
        static constexpr size_t size = 1;
        constexpr std::array<uint8_t, size> encode() const {
            return { fixed_instr_table.at(static_cast<uint8_t>(index)) };
        }
    };



    // ---- Append Bytes ----
    template <typename... Bytes>
    struct ByteInstr {
        static constexpr size_t size = sizeof...(Bytes);

        std::array<uint8_t, size> bytes_array;

        constexpr ByteInstr(Bytes... bytes) : bytes_array{ static_cast<uint8_t>(bytes)... } {}

        constexpr std::array<uint8_t, size> encode() const {
            return bytes_array;
        }
    };



#pragma endregion Instructions

    // ---- Dynamic Assembler Core ----
    template<typename... Instrs>
    std::array<uint8_t, total_size<Instrs...>> assemble(const Instrs&... instrs) {
        static_assert(total_size<Instrs...> < 1024, "Exceeds the stack limit");

        std::array<uint8_t, total_size<Instrs...>> code = {};
        size_t offset = 0;


        auto emit = [&](auto&& instr) {
            auto bytes = instr.encode();
            std::memcpy(code.data() + offset, bytes.data(), bytes.size());
            offset += bytes.size();
            };

        (emit(instrs), ...);
        return code;
    }

    // ---- Static Assembler Core ----
    template <typename... Instrs>
    constexpr auto assemble_static(const Instrs&... instrs) {
        static_assert(total_size<Instrs...> < 1024, "Exceeds the stack limit");
        std::array<uint8_t, total_size<Instrs...>> code{};

        size_t offset = 0;
        auto emit = [&](auto&& instr) constexpr {

            const auto bytes = [&] {
                if constexpr (requires { std::declval<std::remove_reference_t<decltype(instr)>>().encode_constexpr(); }) {
                    return instr.encode_constexpr();
                }
                else {
                    return instr.encode();
                }
            }();


            for (size_t i = 0; i < bytes.size(); ++i)
                code[offset + i] = bytes[i];
            offset += bytes.size();

        };

        (emit(instrs), ...);
        return code;
    }


#pragma region HelperAPI
    template <typename BaseT = nulltype_t, typename IndexT = nulltype_t, typename DispSizeT = nulltype_t>
    struct MemoryBuilder {
        const BaseT base;
        const IndexT index;
        const uint8_t scale;
        const DispSizeT disp;

        template <typename Size>
        constexpr auto build() {
            return Memory<Size, BaseT, IndexT, DispSizeT> {base, index, scale, disp};
        }
    };


    // [RAX + 0x100]
    template <typename BaseT, typename DispSizeT, typename = std::enable_if_t<is_register<BaseT>() && is_imm<DispSizeT>()>>
    constexpr MemoryBuilder<BaseT, nulltype_t, DispSizeT> operator+(BaseT base, DispSizeT disp) {
        return  { base, {}, SCALING_NONE, disp };
    }

    // [RAX - 0x100]
    template <typename BaseT, typename DispSizeT, typename = std::enable_if_t<is_register<BaseT>() && is_imm<DispSizeT>()>>
    constexpr MemoryBuilder<BaseT, nulltype_t, DispSizeT> operator-(BaseT base, DispSizeT disp) {
        return  { base, {}, SCALING_NONE, disp * -1 };
    }

    // [RAX + RBX]
    template <typename BaseT, typename IndexT, typename = std::enable_if_t<is_register<BaseT>() && is_register<IndexT>()>>
    constexpr MemoryBuilder<BaseT, IndexT> operator+(BaseT base, IndexT index) {
        return { base, index, SCALING_NONE, {} };
    }

    // [ (RAX + RBX) + (0x100) ] or [ ( RAX + RBX * SCALING_W ) + (0x100) ]
    template <typename BaseT, typename IndexT, typename DispSizeT, typename = std::enable_if_t<is_register<BaseT>() && is_register<IndexT>() && is_imm<DispSizeT>()>>
    constexpr MemoryBuilder<BaseT, IndexT, DispSizeT> operator+(MemoryBuilder<BaseT, IndexT, nulltype_t> mb, DispSizeT disp) {
        return { mb.base, mb.index, mb.scale, disp };
    }

    // [ (RAX + RBX) - (0x100) ] or [ ( RAX + RBX * SCALING_W ) - (0x100) ]
    template <typename BaseT, typename IndexT, typename DispSizeT, typename = std::enable_if_t<is_register<BaseT>() && is_register<IndexT>() && is_imm<DispSizeT>()>>
    constexpr MemoryBuilder<BaseT, IndexT, DispSizeT> operator-(MemoryBuilder<BaseT, IndexT, nulltype_t> mb, DispSizeT disp) {
        return { mb.base, mb.index, mb.scale, disp * -1 };
    }

    // [RAX * SCALING_W]
    template <typename IndexT, typename = std::enable_if_t<is_register<IndexT>()>>
    constexpr MemoryBuilder<nulltype_t, IndexT, nulltype_t> operator*(IndexT index, ScalingSize scale) {

        return { {}, index, scale, {} };
    }

    // [(RAX * SCALING_W) + 0x100]
    template <typename IndexT, typename DispSizeT, typename = std::enable_if_t<is_register<IndexT>() && is_imm<DispSizeT>()>>
    constexpr MemoryBuilder<nulltype_t, IndexT, DispSizeT> operator+(MemoryBuilder<nulltype_t, IndexT, nulltype_t> mb, DispSizeT disp) {
        return { {}, mb.index, mb.scale, disp };
    }

    // [(RAX * SCALING_W) - 0x100]
    template <typename IndexT, typename DispSizeT, typename = std::enable_if_t<is_register<IndexT>() && is_imm<DispSizeT>()>>
    constexpr MemoryBuilder<nulltype_t, IndexT, DispSizeT> operator-(MemoryBuilder<nulltype_t, IndexT, nulltype_t> mb, DispSizeT disp) {
        return { {}, mb.index, mb.scale, disp * -1 };
    }

    // [RAX + (RBX * SCALING_W)]
    template <typename BaseT, typename IndexT, typename = std::enable_if_t<is_register<BaseT>() && is_register<IndexT>()>>
    constexpr MemoryBuilder<BaseT, IndexT, nulltype_t> operator+(BaseT base, MemoryBuilder<nulltype_t, IndexT, nulltype_t> mb) {
        return { base, mb.index, mb.scale, {} };
    }


    // Avoiding REG * NUM to prevent arithmetic multiplication. (There is a scaling enum instead)
    template <typename RegT, typename = std::enable_if_t<is_register<RegT>()>>
    constexpr void operator*(RegT reg, int imm) {

    }


    template <typename Size>
    struct MemSizeProxy {
        template <typename B, typename I, typename D>
        constexpr auto operator[](MemoryBuilder<B, I, D> builder) const {
            return builder.build<Size>();
        }

        template <typename BaseT>
        constexpr auto operator[](BaseT base) const {

            if constexpr (is_register<BaseT>())
                return Memory<Size, BaseT>{base, {}, SCALING_NONE, {}};
            else
                return Memory<Size, nulltype_t, nulltype_t, BaseT>{{}, {}, SCALING_NONE, base};
        }


    };

    struct MemProxy {
        static constexpr MemSizeProxy<uint8_t>  BYTE{};
        static constexpr MemSizeProxy<uint16_t>  WORD{};
        static constexpr MemSizeProxy<uint32_t> DWORD{};
        static constexpr MemSizeProxy<uint64_t> QWORD{};
    };

    constexpr MemProxy mem{};

    template <typename T1, typename T2>
    constexpr auto mov(T1 type1, T2 type2) {
        return MovInstr<T1, T2> { type1, type2 };
    }
    
    template <typename T1, typename T2>
    constexpr auto lea(T1 type1, T2 type2) {
        return LeaInstr<T1, T2> { type1, type2 };
    }

    template <typename... BytesT>
    constexpr auto append(BytesT&&... bytes) {
        return ByteInstr<std::decay_t<BytesT>...>{std::forward<BytesT>(bytes)...};
    }

    template <typename T1, typename T2>
    constexpr auto add(T1 type1, T2 type2) {
        return ArithmeticInstr<T1, T2> { type1, type2, OP_EXTENSION::OP_ADD };
    }
    
    constexpr FixedSizeInstr ret() {
        return FixedSizeInstr { FixedInstrType::RET };
    }

    constexpr FixedSizeInstr nop() {
        return FixedSizeInstr{ FixedInstrType::NOP };
    }

    constexpr FixedSizeInstr pushf() {
        return FixedSizeInstr{ FixedInstrType::PUSHF };
    }

    constexpr FixedSizeInstr popf() {
        return FixedSizeInstr{ FixedInstrType::POPF };
    }

#pragma endregion HelperAPI
    

} // namespace blazing_asm

#endif // BLAZING_ASSEMBLER_HPP
