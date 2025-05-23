#ifndef BLAZING_ASSEMBLER_HPP
#define BLAZING_ASSEMBLER_HPP

#pragma once

#include <array>
#include <type_traits>

namespace blazing_asm {

    struct nulltype_t {

    };

    // ---- Register Enum ----

    enum class OperandSize : uint8_t { None = 0, Byte = 1, Word = 2, DWord = 4, QWord = 8 };
    enum class OperandType : uint8_t { None = 0, Immediate, Register, Memory };

    enum Reg64 : uint8_t { RAX = 0, RCX, RDX, RBX, RSP, RBP, RSI, RDI };
    enum Reg32 : uint8_t { EAX = 0, ECX, EDX, EBX, ESP, EBP, ESI, EDI };
    enum Reg16 : uint8_t { AX = 0, CX, DX, BX, SP, BP, SI, DI };
    enum Reg8 : uint8_t { AL = 0, CL, DL, BL };

    enum Reg8H : uint8_t { AH = 4, CH, DH, BH }; // REX disabled
    enum Reg8L : uint8_t { SPL = 4, BPL, SIL, DIL }; // REX enabled

    enum RegX  : uint8_t {  R8,  R9,  R10,  R11,  R12,  R13,  R14,  R15 }; // Extended QWORD Registers
    enum RegXD : uint8_t { R8D, R9D, R10D, R11D, R12D, R13D, R14D, R15D }; // Extended DWORD Registers
    enum RegXW : uint8_t { R8W, R9W, R10W, R11W, R12W, R13W, R14W, R15W }; // Extended WORD Registers
    enum RegXB : uint8_t { R8B, R9B, R10B, R11B, R12B, R13B, R14B, R15B }; // Extended BYTE Registers


    // R8-R15 registers
    template<typename T>
    constexpr bool is_reg_r() {
        if constexpr (std::is_same_v<T, RegX> || std::is_same_v<T, RegXD> ||
            std::is_same_v<T, RegXW> || std::is_same_v<T, RegXB>)
            return true;
        return false;
    }

    // R8-R15 registers
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
            std::is_same_v<T, RegXB>)
            return true;
        return false;
    }


    template <typename BaseRegT = nulltype_t, typename IndexRegT = nulltype_t>
    struct Memory {

    };

    template <typename Type>
    constexpr OperandSize get_op_size() {
        if constexpr (std::is_same_v<Type, Reg64> || std::is_same_v<Type, RegX> || std::is_same_v<Type, uint64_t>)
        {
            return OperandSize::QWord;
        }
        if constexpr (std::is_same_v<Type, Reg32> || std::is_same_v<Type, RegXD> || std::is_same_v<Type, uint32_t>)
        {
            return OperandSize::DWord;
        }
        if constexpr (std::is_same_v<Type, Reg16> || std::is_same_v<Type, RegXW> || std::is_same_v<Type, uint16_t>)
        {
            return OperandSize::Word;
        }
        if constexpr (std::is_same_v<Type, Reg8L> || std::is_same_v<Type, Reg8H> || std::is_same_v<Type, Reg8> || std::is_same_v<Type, RegXB> || std::is_same_v<Type, uint8_t>)
        {
            return OperandSize::Byte;
        }
        return OperandSize::None;

    }

    template <typename Type>
    constexpr OperandType get_op_type() {
        if constexpr (std::is_same_v<Type, uint64_t> || std::is_same_v<Type, uint32_t> || std::is_same_v<Type, uint16_t> || std::is_same_v<Type, uint8_t>)
        {
            return OperandType::Immediate;
        }
        if constexpr (is_register<Type>()) {
            return OperandType::Register;
        }
        /*if constexpr (std::is_same_v<Type, Memory>)
        {
            return OperandType::Memory;
        }*/

        return OperandType::None;
    }

    template <typename RegType>
    constexpr bool op_requires_rex() {

        if constexpr (std::is_same_v<RegType, Reg8L> || is_reg_r<RegType>() || std::is_same_v<RegType, Reg64>)
            return true;
        return false;
    }

    template <typename RegDest, typename RegSrc>
    constexpr uint8_t op_calculate_rex() {

        if constexpr (!op_requires_rex<RegDest>() && !op_requires_rex<RegSrc>())
            return 0;
        
        // High 8-bit registers suppress REX
        static_assert(!((op_requires_rex<RegDest>() || op_requires_rex<RegSrc>()) && (std::is_same_v<RegDest, Reg8H> || std::is_same_v<RegSrc, Reg8H>)), "Can't encode R8H registers in an instruction requiring REX prefix");
        

        uint8_t rex = 0x40; // Base REX prefix

        if constexpr (!std::is_same_v<RegSrc, nulltype_t> && (std::is_same_v<RegDest, Reg64> || std::is_same_v<RegDest, RegX>))
            rex |= 0x08; // REX.W

        // R bit (modrm.reg extension - source register)
        if constexpr (is_reg_r<RegSrc>())
            rex |= 0x04;

        // B bit (modrm.rm extension - destination register)
        if constexpr (is_reg_r<RegDest>())
            rex |= 0x01;

        return rex;
    }

    template <typename Type>
    struct OperandHelper {
        static constexpr OperandType type = get_op_type<Type>();
        static constexpr OperandSize size = get_op_size<Type>();
    };

    
    // -- All 32-bit conditional jump opcodes (0F 8X)

    enum class Condition : uint8_t {
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
    };


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



    // ---- MOV ----
    template <typename FirstType, typename SecondType>
    struct MovInstr {
        static constexpr OperandHelper<FirstType> op1;
        static constexpr OperandHelper<SecondType> op2;

        FirstType op1_val;
        SecondType op2_val;

        //static const bool rex;

        static constexpr size_t calc_array_size() {


            static_assert(op1.type != OperandType::Immediate, "MOV: First operand can't be an Immediate");
            static_assert(op1.type != OperandType::None, "MOV: Unknown first operand type");
            static_assert(op2.type != OperandType::None, "MOV: Unknown second operand type");

            static_assert(op1.size != OperandSize::None, "MOV: Unknown first operand size");
            static_assert(op2.size != OperandSize::None, "MOV: Unknown second operand size");

            static_assert(!(op2.type != OperandType::Immediate && op1.size != op2.size), "MOV: Both operands have to be the same size");
            static_assert(!(op2.type == OperandType::Immediate && op1.size < op2.size), "MOV: Immediate is higher than the register");

            // mov reg, imm
            // mov reg, reg
            // mov reg, mem
            // mov mem, imm
            // mov mem, reg

            size_t sz = 1;

            if (op_calculate_rex<FirstType, SecondType>() != 0)
                sz++;

            if (op1.size == OperandSize::Word)
                sz++;

            if (op2.type == OperandType::Immediate) {

                if (op1.size == OperandSize::QWord && op2.size < OperandSize::QWord)
                    sz += size_t(OperandSize::DWord) + 1;
                else
                    sz += size_t(op1.size);

            }
            else if (op2.type == OperandType::Register)
                sz++;
            

            return sz;
        }

        static constexpr size_t size = calc_array_size();


        constexpr std::array<uint8_t, size> encode_header() const {
            std::array<uint8_t, size> out = {};
            size_t index = 0;


            if constexpr (op1.type == OperandType::Register) {


                if constexpr (op1.size == OperandSize::Word)
                    out[index++] = 0x66;

                constexpr uint8_t rex = op_calculate_rex<FirstType, SecondType>();

                if constexpr (rex != 0)
                    out[index++] = rex;

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



                }
            }
            else if constexpr (op1.type == OperandType::Memory) {

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



    // ---- INC reg (single operand) ----
    struct IncR64 {
        Reg64 reg;

        static constexpr size_t size = 3;

        std::array<uint8_t, size> encode() const {
            std::array<uint8_t, size> bytes = {};
            const uint8_t reg_id = static_cast<uint8_t>(reg);
            bytes[0] = 0x48 | ((reg_id >> 3) & 1);       // REX
            bytes[1] = 0xFF;                             // Opcode for inc r/m64
            bytes[2] = 0xC0 | (reg_id & 0x7);            // ModRM for reg
            return bytes;
        }
    };

    // ---- Conditional Jump ----
    /*struct JumpCond {
        Condition cond;
        int32_t offset;

        static constexpr size_t size = 6;

        std::array<uint8_t, size> encode() const {
            std::array<uint8_t, size> bytes = {};
            bytes[0] = 0x0F;
            bytes[1] = static_cast<uint8_t>(cond);
            Write<uint32_t>(bytes.data() + 2, static_cast<uint32_t>(offset));
            return bytes;
        }
    };*/

    // ---- RET ----
    struct Ret {
        static constexpr size_t size = 1;
        constexpr std::array<uint8_t, size> encode() const {
            return { 0xC3 };  // RET opcode
        }
    };


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

    // ---- Helper APIs ----


    template <typename T1, typename T2>
    constexpr auto mov(T1 type1, T2 type2) {
        return MovInstr<T1, T2> { type1, type2 };
    }

    constexpr Ret ret() {
        return Ret{};
    }
    constexpr IncR64 inc(Reg64 r) {
        return IncR64{ r };
    }

    /*constexpr JumpCond jcc(Condition cond, int32_t rel32) {
        return JumpCond{ cond, rel32 };
    }*/

    

} // namespace blazing_asm

#endif // BLAZING_ASSEMBLER_HPP
