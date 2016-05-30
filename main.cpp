#include <stdio.h>
#include "half.h"

int main()
{
	FLOAT_32 af;
	af.f32 = 20001.52255f;

	uint16_t ah = half_from_float(af.u32);

	FLOAT_32 ca;
	ca.u32 = half_to_float(ah);

	printf("Single precision: %f\nHalf precision: %f\n", af.f32, ca.f32);

	return 0;
}