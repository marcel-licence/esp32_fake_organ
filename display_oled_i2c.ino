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
 * @file display_oled_i2c.ino
 * @author Marcel Licence
 * @date 24.09.2021
 *
 * @brief This module is used to create a steady image of the audio signal on your OLED display
 *
 * Define your adc mapping in the lookup table
 *
 * @see ESP32 Fake Organ https://youtu.be/x4WEWTdZR90
 */


#ifdef __CDT_PARSER__
#include <cdt.h>
#endif


#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library.
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


void display_oled_i2c_setup()
{


    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
    {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;); // Don't proceed, loop forever
    }

    // Show initial display buffer contents on the screen --
    // the library initializes this with an Adafruit splash screen.
    display.display();
    delay(2000); // Pause for 2 seconds

    // Clear the buffer
    display.clearDisplay();
    display.display();
}




#define DISP_SAMPLE_LEN (SAMPLE_BUFFER_SIZE * 5)
#define CONV_WIN_LEN    128
#define CONV_OFFSET ((CONV_WIN_LEN - 128) / 2)
#define OLED_CONV_LEN ((DISP_SAMPLE_LEN) - CONV_WIN_LEN)
float dispSamples[DISP_SAMPLE_LEN];
float dispSamples2[DISP_SAMPLE_LEN];
//float dispSamples3[DISP_SAMPLE_LEN];
uint32_t disOffset = 0;

float LastData[CONV_WIN_LEN];

#define OLED_CONV

void display_oled_process(void)
{
    if (disOffset >= DISP_SAMPLE_LEN)
    {
        memcpy(dispSamples2, dispSamples, sizeof(dispSamples2));
        disOffset = 0;
        display_oled_i2c_osc(dispSamples2, DISP_SAMPLE_LEN);
    }
}

void display_add_samples(float *samples, uint32_t len)
{
    if (disOffset < DISP_SAMPLE_LEN)
    {
#if 0
        memcpy(&dispSamples[disOffset], fl_sample, SAMPLE_BUFFER_SIZE * (sizeof(float)));
        disOffset += SAMPLE_BUFFER_SIZE;
#else
        for (int i = 0; (2 * i) < SAMPLE_BUFFER_SIZE; i += 1)
        {
            dispSamples[disOffset + i] = samples[i * 2];
        }
        disOffset += SAMPLE_BUFFER_SIZE / 2;
#endif
    }
}

void display_oled_i2c_osc(float *data, uint32_t len)
{
    static uint8_t drawData[128];

    uint32_t start = 0;

#ifdef OLED_CONV

    static float conv[OLED_CONV_LEN];

    memset(conv, 0, sizeof(conv));
    for (int n = 0; n < CONV_WIN_LEN; n++)
    {
        for (int m = 0; m < OLED_CONV_LEN; m++)
        {
            conv[m] += LastData[n] * data[n + m];
        }
    }

    float maxD;
    maxD = 0;
    for (int n = 0; n < OLED_CONV_LEN; n++)
    {
        if (conv[n] > maxD)
        {
            start = n;
            maxD = conv[n];
        }
    }
    memcpy(LastData, &data[start], sizeof(LastData));

#else

    for (int n = 0; n < len - 130; n++)
    {
        if (data[n] < 0 && data[n + 1] >= 0)
        {
            start = n + 1;
            break;
        }
    }
#endif

    bool proc = false;
    display.clearDisplay();
    for (int n = 0; n < 128; n++)
    {
        drawData[n] = 32 - (LastData[n + CONV_OFFSET] * 128.0f * 0.5f);
        // drawData2[n] = 32 - (dataSync[n + start] * 16.0f);
        if (drawData[n] != 32)
        {
            proc = true;
        }
    }
    if (proc)
    {
#if 0
        for (int n = 0; n < 128; n++)
        {
            display.drawPixel(n, drawData[n], SSD1306_WHITE);
        }
#else
        for (int n = 0; n < 127; n++)
        {
            display.drawLine(n, drawData[n], n + 1, drawData[n + 1], SSD1306_WHITE);
        }
#endif
    }
    display.display();
}
