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
 * @file led_module.ino
 * @author Marcel Licence
 * @date 04.10.2021
 *
 * @brief  This module drives the ws2812 LEDs
 */


#ifdef __CDT_PARSER__
#include <cdt.h>
#endif


#include <Adafruit_NeoPixel.h>

/*
 * I am using a low value to keep the LED's dark
 * and to avoid blindness xD
 */
uint32_t brightness = 64;

/* pixel count of the 8x8 module */
#define NUMPIXELS   (8*8)


struct pixel_rgb
{
    uint8_t r, g, b, a;
};

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, LED_STRIP_PIN,
                          NEO_GRB + NEO_KHZ800);

struct pixel_rgb pixels[NUMPIXELS] = {0};


void LedMatrix_Init(void)
{
    strip.begin();
    strip.show();
    for (int i = 0; i < NUMPIXELS; i++)
    {
        strip.setPixelColor(i, 8, 0, 0);
        strip.show();
        delay(5);
    }
    for (int i = 0; i < NUMPIXELS; i++)
    {
        strip.setPixelColor(i, 0, 8, 0);
        strip.show();
        delay(5);
    }
    for (int i = 0; i < NUMPIXELS; i++)
    {
        strip.setPixelColor(i, 0, 0, 8);
        strip.show();
        delay(5);
    }
    for (int i = 0; i < NUMPIXELS; i++)
    {
        strip.setPixelColor(i, 0, 0, 0);
    }
    strip.show();

    for (int i = 0; i < NUMPIXELS; i++)
    {
        strip.setPixelColor(i, 1, 0, 0);
    }
    strip.show();
}


void LedMatrix_Display(void)
{
    /* nothing todo at the moment */
}


void LedMatrix_SetBrighness(uint8_t unused, float value)
{
    brightness = (value * 255.0f);
}
