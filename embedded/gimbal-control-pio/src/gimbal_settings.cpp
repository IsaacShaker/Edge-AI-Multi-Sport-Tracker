#include "gimbal_settings.h"
#include <EEPROM.h>

static const int EEPROM_ADDR = 0;

//Get the settings that are currently in the struct, then save to EEPROM at magic.
void settings_save(Settings &s) {
    s.magic = SETTINGS_MAGIC;
    s.version = SETTINGS_VERSION;

    EEPROM.put(EEPROM_ADDR, s);
}

//Check if magic number is valid. If not, use the default settings.
bool settings_load(Settings &s) {
    EEPROM.get(EEPROM_ADDR, s);

    if (s.magic != SETTINGS_MAGIC) {
        return false;
    }

    if (s.version != SETTINGS_VERSION) {
        return false;
    }

    return true;
}