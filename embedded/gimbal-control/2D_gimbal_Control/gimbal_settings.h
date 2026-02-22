#include <Arduino.h>

#define SETTINGS_MAGIC   0xABCD1234
#define SETTINGS_VERSION 1

struct Settings {
    float bottom_p;
    float bottom_i;
    float bottom_d;

    float top_p;
    float top_i;
    float top_d;

    float bottom_home;
    float top_home;

    float top_vlimit;
    float bottom_vlimit;

    float top_lpf;
    float bottom_lpf;

    uint32_t magic;
    uint32_t version;
};

//commands to save and load.
void settings_save(Settings &s);
bool settings_load(Settings &s);