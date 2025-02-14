//
// Created by Kristoffer on 07.09.2024.
//

#include "wasm_utils.h"

extern "C" {
    double multiply(const double a, const double b)
    {
        return a * b;
    }
}