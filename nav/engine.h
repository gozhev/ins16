#ifndef _ENGINE_
#define _ENGINE_

#include <stdint.h>

#define SAMPLING_FREQUENCY 1000 /* Hz */

typedef int (*pfn_obtain_sample)
		(int16_t *, int16_t *, int16_t *,
		 int16_t *, int16_t *, int16_t *, void *);

void engine_set_callback_functions (pfn_obtain_sample, void *);
void engine_set_calculation_params (double calibration_time);
int engine_calculate ();

#endif
