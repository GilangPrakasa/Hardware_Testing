#include "ElegantOTA.h"

// #include "DeviceConfig.h"
// #include "avrOTAWebpage.h"
// #include "eventbus.h"
// #include "minimalWebpage.h"
#include "storage.h"

static const char *TAG = "ElegantOTA";

#define AVRPROG_SCK 18
#define AVRPROG_MISO 19
#define AVRPROG_MOSI 23
#define AVRPROG_RESET 25

// static uint32_t counter;
static File file;

Adafruit_AVRProg *avrprog;

extern const image_t *images[];

String response = "";

ElegantOtaClass::ElegantOtaClass() {
    this->setID(String((uint32_t)ESP.getEfuseMac(), HEX).c_str());
}

void ElegantOtaClass::setID(const char *id) {
    snprintf(_id, sizeof(_id), "{ \"id\": \"%s\", \"hardware\": \"ESP32\" }", id);
}

void ElegantOtaClass::setupProg() {
    SPI.end();
    avrprog->setSPI(AVRPROG_RESET, AVRPROG_SCK, AVRPROG_MOSI, AVRPROG_MISO);
    // avrprog->setSPI(AVRPROG_RESET, &SPI);
}

void ElegantOtaClass::updateController(uint8_t *version) {
    for (uint8_t i = 0; i < versionLen; i++) {
        if (controllerVersion[i] != version[i]) {
            ESP_LOGI(TAG, "update version from %d.%d.%d to %d.%d.%d",
                     controllerVersion[0], controllerVersion[1], controllerVersion[2],
                     version[0], version[1], version[2]);
            flashImage();
        }
    }
    ESP_LOGI(TAG, "version up to date");
    return;
}

void ElegantOtaClass::flashImage() {
    avrprog = new Adafruit_AVRProg();
    setupProg();
    response = uploadProg();
    // Serial.println(response);
    delete avrprog;
    // ESP.restart();
}

String ElegantOtaClass::uploadProg() {
    // eventbus_post(__func__, EFEEDER_FW_UPGRADING, NULL);

    if (!avrprog->targetPower(true)) {
        avrprog->error("Failed to connect to target");
        return "Failed to connect";
    }

    Serial.print(F("\nReading signature: "));
    uint16_t signature = avrprog->readSignature();
    Serial.println(signature, HEX);
    if (signature == 0 || signature == 0xFFFF) {
        avrprog->error(F("No target attached?"));
        return "No target";
    }

    const image_t *targetimage = images[0];
    if (targetimage->image_chipsig != signature) {
        avrprog->error(F("Signature doesn't match image"));
        return "Wrong signature";
    }
    Serial.println(F("Found matching chip/image"));

    Serial.print(F("Erasing chip..."));
    avrprog->eraseChip();
    Serial.println(F("Done!"));

    if (!avrprog->programFuses(
            targetimage->image_progfuses)) {  // get fuses ready to program
        avrprog->error(F("Programming Fuses fail"));
        return "Fuses program fail";
    }

    if (!avrprog->verifyFuses(targetimage->image_progfuses,
                              targetimage->fusemask)) {
        avrprog->error(F("Failed to verify fuses"));
        return "Fuses verification fail";
    }

    if (!avrprog->writeImage(targetimage->image_hexcode,
                             pgm_read_byte(&targetimage->image_pagesize),
                             pgm_read_word(&targetimage->chipsize))) {
        avrprog->error(F("Failed to write flash"));
        return "Flash write fail";
    }

    Serial.println(F("\nVerifing flash..."));
    if (!avrprog->verifyImage(targetimage->image_hexcode)) {
        avrprog->error(F("Failed to verify flash"));
        return "Flash verify fail";
    }
    Serial.println(F("\nFlash verified correctly!"));

    // Set fuses to 'final' state
    if (!avrprog->programFuses(targetimage->image_normfuses)) {
        avrprog->error("Programming fuses fail");
        return "Fuses final program fail";
    }

    if (!avrprog->verifyFuses(targetimage->image_normfuses,
                              targetimage->fusemask)) {
        avrprog->error("Failed to verify fuses");
        return "Fuses final verify fail";
    } else {
        Serial.println("Fuses verified correctly!");
        return "Flash AVR Berhasil";
    }
}

ElegantOtaClass ElegantOTA;
