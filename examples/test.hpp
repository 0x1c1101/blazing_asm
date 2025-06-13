#pragma once
#include <iostream>
#include <random>
#include <iomanip>
#include "Windows.h"



void print_bytes(auto& bytes) {
    std::cout << std::hex << std::uppercase;
    for (auto& byte : bytes) {
        std::cout << "0x" << std::setw(2) << std::setfill('0') << int(byte) << " ";
    }
    std::cout << std::endl;
}

class Test {
public:
	void str_decryptor();
	void arr_multiplier();

};