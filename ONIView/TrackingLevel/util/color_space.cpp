#include "color_space.h"

void UInt16RGB2Jet(unsigned short depth, unsigned short min_value,
		unsigned short max_value, unsigned char* rgb) {
	float value = (float) (depth - min_value) / (float) (max_value - min_value);

	float fourValue = 4 * value;
	float red = min(fourValue - 1.5, -fourValue + 4.5);
	float green = min(fourValue - 0.5, -fourValue + 3.5);
	float blue = min(fourValue + 0.5, -fourValue + 2.5);

	red = (red < 0) ? 0.0f : red;
	red = (red > 1.0) ? 1.0f : red;
	green = (green < 0) ? 0.0f : green;
	green = (green > 1.0) ? 1.0f : green;
	blue = (blue < 0) ? 0.0f : blue;
	blue = (blue > 1.0) ? 1.0f : blue;

	rgb[0] = (unsigned char) (blue * 255);
	rgb[1] = (unsigned char) (green * 255);
	rgb[2] = (unsigned char) (red * 255);

	return;
}


void UInt16RGB2Jet(unsigned short depth, unsigned short min_value,
		unsigned short max_value, float* rgb) {

	float value = (float) (depth - min_value) / (float) (max_value - min_value);

	float fourValue = 4 * value;
	float red = min(fourValue - 1.5, -fourValue + 4.5);
	float green = min(fourValue - 0.5, -fourValue + 3.5);
	float blue = min(fourValue + 0.5, -fourValue + 2.5);

	red = (red < 0) ? 0.0f : red;
	red = (red > 1.0) ? 1.0f : red;
	green = (green < 0) ? 0.0f : green;
	green = (green > 1.0) ? 1.0f : green;
	blue = (blue < 0) ? 0.0f : blue;
	blue = (blue > 1.0) ? 1.0f : blue;

	rgb[0] = blue;
	rgb[1] = green;
	rgb[2] = red;

	return;

}

void FloatRGB2Jet(float depth, float min_value,
		float max_value, unsigned char* rgb) {
	float value = (float) (depth - min_value) / (float) (max_value - min_value);

	float fourValue = 4 * value;
	float red = min(fourValue - 1.5, -fourValue + 4.5);
	float green = min(fourValue - 0.5, -fourValue + 3.5);
	float blue = min(fourValue + 0.5, -fourValue + 2.5);

	red = (red < 0) ? 0.0f : red;
	red = (red > 1.0) ? 1.0f : red;
	green = (green < 0) ? 0.0f : green;
	green = (green > 1.0) ? 1.0f : green;
	blue = (blue < 0) ? 0.0f : blue;
	blue = (blue > 1.0) ? 1.0f : blue;

	rgb[0] = (unsigned char) (blue * 255);
	rgb[1] = (unsigned char) (green * 255);
	rgb[2] = (unsigned char) (red * 255);

	return;
}

