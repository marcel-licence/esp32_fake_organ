/*
 * Copyright (c) 2021 Marcel Licence
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Dieses Programm ist Freie Software: Sie können es unter den Bedingungen
 * der GNU General Public License, wie von der Free Software Foundation,
 * Version 3 der Lizenz oder (nach Ihrer Wahl) jeder neueren
 * veröffentlichten Version, weiter verteilen und/oder modifizieren.
 *
 * Dieses Programm wird in der Hoffnung bereitgestellt, dass es nützlich sein wird, jedoch
 * OHNE JEDE GEWÄHR,; sogar ohne die implizite
 * Gewähr der MARKTFÄHIGKEIT oder EIGNUNG FÜR EINEN BESTIMMTEN ZWECK.
 * Siehe die GNU General Public License für weitere Einzelheiten.
 *
 * Sie sollten eine Kopie der GNU General Public License zusammen mit diesem
 * Programm erhalten haben. Wenn nicht, siehe <https://www.gnu.org/licenses/>.
 */

/**
 * @file esp32_fake_organ.ino
 * @author Marcel Licence
 * @date 12.10.2021
 *
 * @brief This is the project file for the esp32_fake_organ project
 *
 * @see https://youtu.be/x4WEWTdZR90
 * @see https://circuits4you.com/2018/12/31/esp32-devkit-esp32-wroom-gpio-pinout/
 */


#ifdef __CDT_PARSER__
#include <cdt.h>
#endif


#include "config.h"

/*
 * required include files
 * add also includes used for other modules
 * otherwise arduino generated declaration may cause errors
 */
#include <SPI.h>
#include <Wire.h>
#include <Arduino.h>
#include <FS.h>
#include <LITTLEFS.h>
#include <SD_MMC.h>
#include <WiFi.h>


void App_UsbMidiShortMsgReceived(uint8_t *msg)
{
    Midi_SendShortMessage(msg);
    Midi_HandleShortMsg(msg, 8);
}

void setup()
{
    /*
     * this code runs once
     */
    delay(500);

    Serial.begin(115200);

    Serial.println();

    /* little note for the license */
    Serial.printf("esp32_fake_organ - Copyright (c) 2021  Marcel Licence\n");
    Serial.printf("This program comes with ABSOLUTELY NO WARRANTY;\n");
    Serial.printf("This is free software, and you are welcome to redistribute it\n");
    Serial.printf("under certain conditions;\n");

    Delay_Init();

    Serial.printf("Initialize Synth Module\n");
    Synth_Init();
    Serial.printf("Initialize I2S Module\n");

    // setup_reverb();

#ifdef BLINK_LED_PIN
    Blink_Setup();
#endif

#ifdef ESP32_AUDIO_KIT
#ifdef ES8388_ENABLED
    ES8388_Setup();
#else
    ac101_setup();
#endif
#endif

    setup_i2s();
    Serial.printf("Initialize Midi Module\n");

    /*
     * setup midi module / rx port
     */
    Midi_Setup();

    Serial.printf("Turn off Wifi/Bluetooth\n");
#if 0
    setup_wifi();
#else
    WiFi.mode(WIFI_OFF);
#endif

#ifndef ESP8266
    btStop();
    // esp_wifi_deinit();
#endif


    Reverb_Setup();

    Serial.printf("ESP.getFreeHeap() %d\n", ESP.getFreeHeap());
    Serial.printf("ESP.getMinFreeHeap() %d\n", ESP.getMinFreeHeap());
    Serial.printf("ESP.getHeapSize() %d\n", ESP.getHeapSize());
    Serial.printf("ESP.getMaxAllocHeap() %d\n", ESP.getMaxAllocHeap());

    Serial.printf("Firmware started successfully\n");

#if 0 /* activate this line to get a tone on startup to test the DAC */
    Synth_NoteOn(0, 64, 1.0f);
#endif

    Core0TaskInit();
}

/*
 * Core 0
 */
/* this is used to add a task to core 0 */
TaskHandle_t Core0TaskHnd;

inline
void Core0TaskInit()
{
    /* we need a second task for the terminal output */
    xTaskCreatePinnedToCore(Core0Task, "CoreTask0", 8000, NULL, 999, &Core0TaskHnd, 0);
}

inline
void Core0TaskSetup()
{
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_CS);
    ScanI2C();

#ifdef BOARD_ML_V1
    MCP23017_TestCS();
#endif

#ifdef OLED_OSC_DISP_ENABLED
    display_oled_i2c_setup();
#endif

#ifdef SPI_DISP_ENABLED
    Display_Test();
#endif

    /*
     * init your stuff for core0 here
     */
#ifdef LED_MATRIX_ENABLED
    LedMatrix_Init();
#endif
#ifdef ADC_TO_MIDI_ENABLED
    AdcMul_Init();
#endif

#ifdef MIDI_VIA_USB_ENABLED
    UsbMidi_Setup();
#endif
    Status_Setup();
}

#ifdef ADC_TO_MIDI_ENABLED
static uint8_t adc_prescaler = 0;
#endif

void Core0TaskLoop()
{
    /*
     * put your loop stuff for core0 here
     */
#ifdef ADC_TO_MIDI_ENABLED
#ifdef MIDI_VIA_USB_ENABLED
    adc_prescaler++;
    if (adc_prescaler > 15) /* use prescaler when USB is active because it is very time consuming */
#endif /* MIDI_VIA_USB_ENABLED */
    {
        adc_prescaler = 0;
        AdcMul_Process();
    }
#endif /* ADC_TO_MIDI_ENABLED */

#ifdef LED_MATRIX_ENABLED
    LedMatrix_Display();
#endif

    Status_Process();

#ifdef SPI_DISP_ENABLED
    Display_Process();
#endif

#ifdef OLED_OSC_DISP_ENABLED
    display_oled_process();
#endif

#ifdef MIDI_VIA_USB_ENABLED
    UsbMidi_Loop();
#endif
#ifdef BOARD_ML_V1
    MCP23_Loop();
#endif
}

void Core0Task(void *parameter)
{
    Core0TaskSetup();

    while (true)
    {
        Core0TaskLoop();

        /* this seems necessary to trigger the watchdog */
        delay(1);
        yield();
    }
}

/*
 * use this if something should happen every second
 * - you can drive a blinking LED for example
 */
inline void Loop_1Hz(void)
{
#ifdef BLINK_LED_PIN
    Blink_Process();
#endif
}


/*
 * our main loop
 * - all is done in a blocking context
 * - do not block the loop otherwise you will get problems with your audio
 */
static float fl_sample[SAMPLE_BUFFER_SIZE];
static float fr_sample[SAMPLE_BUFFER_SIZE];

void loop()
{
    static uint32_t loop_cnt_1hz;
    static uint8_t loop_count_u8 = 0;

    loop_count_u8++;

    loop_cnt_1hz += SAMPLE_BUFFER_SIZE;
    if (loop_cnt_1hz >= (SAMPLE_RATE))
    {
        Loop_1Hz();
        loop_cnt_1hz = 0;
    }

    if (i2s_write_stereo_samples_buff(fl_sample, fr_sample, SAMPLE_BUFFER_SIZE))
    {
        /* nothing for here */
    }

#ifdef OLED_OSC_DISP_ENABLED
    /*
     * for oled display
     */
    display_add_samples(fl_sample, SAMPLE_BUFFER_SIZE);
#endif

    for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++)
    {
        Synth_Process(&fl_sample[i], &fr_sample[i]);
        /*
         * process delay line
         */
        Delay_Process(&fl_sample[i], &fr_sample[i]);
    }

    Reverb_Process(fl_sample, SAMPLE_BUFFER_SIZE);
    memcpy(fr_sample,  fl_sample, sizeof(fr_sample));

    //pll_prozess(&pll_data_buffer, fr_sample, SAMPLE_BUFFER_SIZE, fl_sample); // TODO very very slow


    //ReverbSc_Process(fl_sample, fr_sample, &fl_sample, &fr_sample);

    /*
     * Midi does not required to be checked after every processed sample
     * - we divide our operation by 8
     */
    //if (loop_count_u8 % 8 == 0)
    {
        Midi_Process();
#ifdef MIDI_VIA_USB_ENABLED
        UsbMidi_ProcessSync();
#endif
    }
}


/*
 * Test functions
 */

void  ScanI2C(void)
{

    Wire.begin(21, 22);

    byte error, address;
    int nDevices;

    Serial.println("Scanning...");

    nDevices = 0;
    for (address = 1; address < 127; address++)
    {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0)
        {
            Serial.print("I2C device found at address 0x");
            if (address < 16)
            {
                Serial.print("0");
            }
            Serial.print(address, HEX);
            Serial.println("  !");

            nDevices++;
        }
        else if (error == 4)
        {
            Serial.print("Unknown error at address 0x");
            if (address < 16)
            {
                Serial.print("0");
            }
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0)
    {
        Serial.println("No I2C devices found\n");
    }
    else
    {
        Serial.println("done\n");
    }
}

