#ifndef RDA5807M_H
#define RDA5807M_H

#include <stdint.h>
#include <stdbool.h>

#define FREQ_MIN 87.0f
#define FREQ_MAX 108.0f

void rda5807_init(void);
void rda5807_set_frequency(float freq);
void rda5807_power_down(void);

int rda5807_get_rssi(void);
bool rda5807_get_stereo(void);
void rda5807_get_telemetry(int *rssi, bool *fm_true, bool *stereo);

#endif // RDA5807M_H
