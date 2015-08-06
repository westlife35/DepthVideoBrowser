#pragma once

#include <vector>
#include <cstdio>
#include <cstdlib>
#include <cmath>
using namespace std;

void UInt16RGB2Jet(unsigned short depth, unsigned short min_value,
		unsigned short max_value, unsigned char* rgb);

void UInt16RGB2Jet(unsigned short depth, unsigned short min_value,
		unsigned short max_value, float* rgb);

void FloatRGB2Jet(float depth, float min_value,
		float max_value, unsigned char* rgb);
