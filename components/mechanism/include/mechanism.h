#ifndef CANSAT_MECHANISM_H
#define CANSAT_MECHANISM_H

#include "esp_err.h"

esp_err_t mechanism_init(void);
esp_err_t mechanism_arm(void);
esp_err_t mechanism_fire(void);
void mechanism_force_safe(void);

#endif
