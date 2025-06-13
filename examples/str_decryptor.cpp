#include "test.hpp"

#include "blazing_asm.hpp"
#include "pch.h"

#include <string>

constexpr auto generate_str() {
    const char* str = "Hello Blazing ASM!";
    constexpr size_t size = 19;

    std::array<uint8_t, size> arr;
    for (size_t i = 0; i < size; ++i) {
        arr[i] = str[i] ^ uint8_t(i + 2);
    }

    return arr;
}


void Test::str_decryptor() {

    using namespace basm;

    // Encrypted at compile time
    constexpr auto str = generate_str();

    std::cout << "Encrypted String: ";
    std::string encstr(str.begin(), str.end());
    std::cout << encstr << std::endl;

    // We can safely use the first 4 x64 registers. (RAX, RCX, RDX, RBX)
    uint8_t rand_val = rand() % 4;
    auto reg_i = Reg64(rand_val);
    auto reg_temp = Reg64((rand_val + 1) % 4);
    auto reg_tempbyte = Reg8L(6 + rand() % 2);
    auto reg_ptr = Reg64((rand_val + 2) % 4);

    constexpr auto loop = Label(0);
    constexpr auto done = Label(1);

    // Dynamic decoder stub
    auto shellcode = assemble(
        PUSH(reg_ptr),
        PUSH(reg_i),
        PUSH(reg_temp),
        MOV(reg_ptr, reinterpret_cast<uint64_t>(str.data())),
        XOR(reg_i, reg_i),

        BIND(loop), // .loop:
        CMP(reg_i, uint8_t(19)),
        JCC(JGE, done),
        MOV(reg_tempbyte, MEM.BYTE[reg_ptr + reg_i]),
        MOV(reg_temp, reg_i),
        ADD(reg_temp, uint8_t(2)),
        XOR(reg_tempbyte, reg_lower_8(reg_temp)),
        MOV(MEM.BYTE[reg_ptr + reg_i], reg_tempbyte),
        ADD(reg_i, uint8_t(1)),
        JMP(loop),

        BIND(done), // .done:
        POP(reg_temp),
        POP(reg_i),
        POP(reg_ptr),
        RET()
    );

    std::cout << "Shellcode: ";
    print_bytes(shellcode);
    std::cout << std::endl;

    void* exec = VirtualAlloc(
        nullptr,
        shellcode.size(),
        MEM_COMMIT | MEM_RESERVE,
        PAGE_EXECUTE_READWRITE
    );

    if (!exec) {
        std::cerr << "VirtualAlloc failed!\n";
        exit(EXIT_FAILURE);
    }

    memcpy(exec, shellcode.data(), shellcode.size());

    auto func = reinterpret_cast<void(*)()>(exec);
    func();

    VirtualFree(exec, 0, MEM_RELEASE);

    std::string res(str.begin(), str.end());
    std::cout << "Decrypted String: " << res << std::endl;

    system("PAUSE");
    
}