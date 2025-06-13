#include "test.hpp"

#include "blazing_asm.hpp"
#include "pch.h"

#include <string>


void Test::arr_multiplier() {

    using namespace basm;

    std::array<int, 4> arr = { 0x10, 0x10, 0x10, 0x10 };

    std::cout << "Array: ";
    print_bytes(arr);
    std::cout << std::endl;

    constexpr auto loop = Label(0);
    constexpr auto done = Label(1);

    auto shellcode = assemble(
        PUSH(RBP),
        MOV(RBP, RSP),

        MOV(R10, RDX),
        TEST(R10, R10),
        JCC(JZ, done),
        LEA(R13, MEM.QWORD[RCX]),

        BIND(loop), // .loop

        XOR(RAX, RAX),
        MOV(EAX, MEM.DWORD[R13]),
        IMUL(RAX, R8),
        MOV(MEM.DWORD[R13], EAX),
        ADD(R13, uint8_t(4)),
        SUB(R10, uint8_t(1)),
        TEST(R10, R10),
        JCC(JNZ, loop),

        BIND(done), // .done

        POP(RBP),
        XOR(RAX, RAX),
        RET(),
        MOV(RAX, MEM.QWORD[RSP + uint32_t(rand() % UINT32_MAX)])
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

    auto func = reinterpret_cast<void(*)(uint8_t* ptr, size_t len, size_t multiplier)>(exec);
    func((uint8_t*)arr.data(), arr.size(), 5);

    VirtualFree(exec, 0, MEM_RELEASE);

    std::cout << "New Array: ";
    print_bytes(arr);
    std::cout << std::endl;

    system("PAUSE");

}