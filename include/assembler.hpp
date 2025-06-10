#ifndef BLAZING_ASSEMBLER_HPP
#define BLAZING_ASSEMBLER_HPP

#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <type_traits>
#include <utility>
#include <tuple>

namespace blazing_asm {

    // ---- Register Enum ----
    enum Reg64 : uint8_t {
        RAX = 0, RCX, RDX, RBX,
        RSP, RBP, RSI, RDI,
        R8, R9, R10, R11,
        R12, R13, R14, R15
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


    inline void write64(uint8_t* dest, uint64_t val) {
        // Unrolled store for 8-byte immediate
        dest[0] = val & 0xFF;
        dest[1] = (val >> 8) & 0xFF;
        dest[2] = (val >> 16) & 0xFF;
        dest[3] = (val >> 24) & 0xFF;
        dest[4] = (val >> 32) & 0xFF;
        dest[5] = (val >> 40) & 0xFF;
        dest[6] = (val >> 48) & 0xFF;
        dest[7] = (val >> 56) & 0xFF;
    }

    inline void write32(uint8_t* dest, uint32_t val) {
        dest[0] = val & 0xFF;
        dest[1] = (val >> 8) & 0xFF;
        dest[2] = (val >> 16) & 0xFF;
        dest[3] = (val >> 24) & 0xFF;
    }

    // ---- MOV reg, imm64 ----
    struct MovR64Imm64 {
        Reg64 reg;
        uint64_t imm;

        static constexpr size_t size = 10;

        std::array<uint8_t, size> encode() const {
            std::array<uint8_t, size> bytes = {};
            const uint8_t reg_id = static_cast<uint8_t>(reg);
            bytes[0] = 0x48 | ((reg_id >> 3) & 1);       // REX prefix
            bytes[1] = 0xB8 | (reg_id & 0x7);            // Opcode + reg

            write64(bytes.data() + 2, imm);
            return bytes;
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
    struct JumpCond {
        Condition cond;
        int32_t offset;

        static constexpr size_t size = 6;

        std::array<uint8_t, size> encode() const {
            std::array<uint8_t, size> bytes = {};
            bytes[0] = 0x0F;
            bytes[1] = static_cast<uint8_t>(cond);
            write32(bytes.data() + 2, static_cast<uint32_t>(offset));
            return bytes;
        }
    };

    // ---- RET ----
    struct Ret {
        static constexpr size_t size = 1;
        std::array<uint8_t, size> encode() const {
            return { 0xC3 };  // RET opcode
        }
    };

    // ---- Instruction Size Helper ----
    template<typename T>
    struct instr_size {
        static constexpr size_t value = T::size;
    };

    template<typename... Instrs>
    constexpr size_t total_size = (instr_size<Instrs>::value + ...);

    // ---- Assembler Core ----
    template<typename... Instrs>
    std::array<uint8_t, total_size<Instrs...>> assemble(const Instrs&... instrs) {
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

    // ---- API Helpers ----
    inline MovR64Imm64 mov(Reg64 r, uint64_t imm) {
        return MovR64Imm64{ r, imm };
    }

    inline IncR64 inc(Reg64 r) {
        return IncR64{ r };
    }

    inline JumpCond jcc(Condition cond, int32_t rel32) {
        return JumpCond{ cond, rel32 };
    }

    inline Ret ret() {
        return Ret{};
    }

} // namespace blazing_asm

#endif // BLAZING_ASSEMBLER_HPP
