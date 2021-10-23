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
 * @file simple_delay.ino
 * @author Marcel Licence
 * @date 04.10.2021
 *
 * @brief This is a simple implementation of a delay line
 * - level adjustable
 * - feedback
 * - length adjustable
 */


#ifdef __CDT_PARSER__
#include <cdt.h>
#endif


//#define DELAY_STEREO

/* max delay can be changed but changes also the memory consumption */
#define MAX_DELAY   11100

// #define DELAY_SAMPLE_FORMAT_INT16
#define DELAY_SAMPLE_FORMAT_FLOAT


#ifdef  DELAY_SAMPLE_FORMAT_INT16
#define DELAY_SAMPLE_FORMAT int16_t
#endif


#ifdef  DELAY_SAMPLE_FORMAT_FLOAT
#define DELAY_SAMPLE_FORMAT float
#endif


/*
 * module variables
 */
DELAY_SAMPLE_FORMAT *delayLine_l;
#ifdef DELAY_STEREO
DELAY_SAMPLE_FORMAT *delayLine_r;
#endif
float delayToMix = 0;
float delayInLvl = 1.0f;
float delayFeedback = 0;
uint32_t delayLen = MAX_DELAY - 2;
uint32_t delayIn = 0;
uint32_t delayOut = 0;

void Delay_Init(void)
{
    delayLine_l = (DELAY_SAMPLE_FORMAT *)malloc(sizeof(DELAY_SAMPLE_FORMAT) * MAX_DELAY);
    if (delayLine_l == NULL)
    {
        Serial.printf("No more heap memory to initiliaze delay line!\n");
    }
#ifdef DELAY_STEREO
    delayLine_r = (DELAY_SAMPLE_FORMAT *)malloc(sizeof(DELAY_SAMPLE_FORMAT) * MAX_DELAY);
    if (delayLine_r == NULL)
    {
        Serial.printf("No more heap memory!\n");
    }
#endif
    Delay_Reset();
}

void Delay_Reset(void)
{
    for (int i = 0; i < MAX_DELAY; i++)
    {
        delayLine_l[i] = 0;
#ifdef DELAY_STEREO
        delayLine_r[i] = 0;
#endif
    }
}

void Delay_Process(float *signal_l, float *signal_r)
{
#ifdef DELAY_SAMPLE_FORMAT_FLOAT
    delayLine_l[delayIn] = *signal_l;
#ifdef DELAY_STEREO
    delayLine_r[delayIn] = *signal_r;
#endif
    delayOut = delayIn + (1 + MAX_DELAY - delayLen);

    if (delayOut >= MAX_DELAY)
    {
        delayOut -= MAX_DELAY;
    }

    *signal_l += delayLine_l[delayOut] * delayToMix;
#ifdef DELAY_STEREO
    *signal_r += delayLine_r[delayOut] * delayToMix;
#endif
    delayLine_l[delayIn] += delayLine_l[delayOut] * delayFeedback;
#ifdef DELAY_STEREO
    delayLine_r[delayIn] += delayLine_r[delayOut] * delayFeedback;
#endif
#endif /* (DELAY_SAMPLE_FORMAT==float) */

#ifdef DELAY_SAMPLE_FORMAT_INT16
    delayLine_l[delayIn] = (((float)0x8000) * *signal_l * delayInLvl);
#ifdef DELAY_STEREO
    delayLine_r[delayIn] = (((float)0x8000) * *signal_r * delayInLvl);
#endif
    delayOut = delayIn + (1 + MAX_DELAY - delayLen);

    if (delayOut >= MAX_DELAY)
    {
        delayOut -= MAX_DELAY;
    }

    *signal_l += ((float)delayLine_l[delayOut]) * delayToMix / ((float)0x8000);
#ifdef DELAY_STEREO
    *signal_r += ((float)delayLine_r[delayOut]) * delayToMix / ((float)0x8000);
#endif
    delayLine_l[delayIn] += (((float)delayLine_l[delayOut]) * delayFeedback);
#ifdef DELAY_STEREO
    delayLine_r[delayIn] += (((float)delayLine_r[delayOut]) * delayFeedback);
#endif
#endif /* (DELAY_SAMPLE_FORMAT==int16_t) */
    delayIn ++;

    if (delayIn >= MAX_DELAY)
    {
        delayIn = 0;
    }
}

void Delay_SetFeedback(uint8_t unused, float value)
{
    delayFeedback = value;
    Status_ValueChangedFloat("DelayFeedback", value);
}

void Delay_SetLevel(uint8_t unused, float value)
{
    delayToMix = value;
    Status_ValueChangedFloat("DelayOutputLevel", value);
}

void Delay_SetLength(uint8_t unused, float value)
{
    delayLen = (uint32_t)(((float)MAX_DELAY - 1.0f) * value);
    Status_ValueChangedFloat("DelayLenMs", delayLen * (1000.0f / ((float)SAMPLE_RATE)));
}
