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
 * @file add_synth_module.ino.ino
 * @author Marcel Licence
 * @date 15.10.2021
 *
 * @brief Implementation of a simple polyphonic synthesizer module
 * - it supports different waveforms
 * - it supports polyphony
 * - implemented ADSR for velocity and filter
 * - allows usage of multiple oscillators per voice
 *
 * @see https://youtu.be/x4WEWTdZR90
 */


#ifdef __CDT_PARSER__
#include "cdt.h"
#endif

/*
 * activate the following macro to enable unison mode
 * by default the saw wave form will be used
 * the waveform controllers are remapped to
 * - waveform1 -> detune
 * - waveform2 -> oscillator count
 */

/*
 * Param indices for Synth_SetParam function
 */
#define SYNTH_PARAM_VEL_ENV_ATTACK  0
#define SYNTH_PARAM_VEL_ENV_DECAY   1
#define SYNTH_PARAM_VEL_ENV_SUSTAIN 2
#define SYNTH_PARAM_VEL_ENV_RELEASE 3
#define SYNTH_PARAM_FIL_ENV_ATTACK  4
#define SYNTH_PARAM_FIL_ENV_DECAY   5
#define SYNTH_PARAM_FIL_ENV_SUSTAIN 6
#define SYNTH_PARAM_FIL_ENV_RELEASE 7
#define SYNTH_PARAM_WAVEFORM_1      8

#define SYNTH_PARAM_MAIN_FILT_CUTOFF    10
#define SYNTH_PARAM_MAIN_FILT_RESO      11
#define SYNTH_PARAM_VOICE_FILT_RESO     12
#define SYNTH_PARAM_VOICE_NOISE_LEVEL   13

#define MAX_POLY_VOICE  7 /* max single voices, can use multiple osc */
#define MAX_POLY_OSC    (MAX_POLY_VOICE*9) /* osc polyphony, always active reduces single voices max poly */

struct channel_settings_s
{
    float drawbar[9];
    float perc[9];
    int dbOffset[9];
    float percSig;
    float percRel;
    uint8_t sel_bar;
    uint8_t percNote;

    float *selectedWaveForm;
};

struct channel_settings_s chCfg =
{
    {1, 1, 1, 1, 0, 0, 0, 0, 0},
    {1, 1, 1, 1, 1, 1, 1, 1, 1},
    {0, 12 + 7, 12, 12 + 7 + 5, 12 + 7 + 5 + 7, 12 + 7 + 5 + 7 + 5, 12 + 7 + 5 + 7 + 5 + 4, 12 + 7 + 5 + 7 + 5 + 4 + 3, 12 + 7 + 5 + 7 + 5 + 4 + 3 + 5},
    0,
    0.999995f,
    0xFF,
    0xFF,
    NULL,
};

static float clk_l;

/*
 * this is just a kind of magic to go through the waveforms
 * - WAVEFORM_BIT sets the bit length of the pre calculated waveforms
 */
#define WAVEFORM_BIT    10UL
#define WAVEFORM_CNT    (1<<WAVEFORM_BIT)
#define WAVEFORM_Q4     (1<<(WAVEFORM_BIT-2))
#define WAVEFORM_MSK    ((1<<WAVEFORM_BIT)-1)
#define WAVEFORM_I(i)   ((i) >> (32 - WAVEFORM_BIT)) & WAVEFORM_MSK


#define MIDI_NOTE_CNT 128
uint32_t midi_note_to_add[MIDI_NOTE_CNT]; /* lookup to playback waveforms with correct frequency */


/*
 * set the correct count of available waveforms
 */
#define WAVEFORM_TYPE_COUNT 7

/*
 * add here your waveforms
 */
float *sine = NULL;
float *saw = NULL;
float *square = NULL;
float *pulse = NULL;
float *tri = NULL;
float *crappy_noise = NULL;
float *silence = NULL;

/*
 * do not forget to enter the waveform pointer addresses here
 */
float *waveFormLookUp[WAVEFORM_TYPE_COUNT];

/*
 * pre selected waveforms
 */



struct adsrT
{
    float a;
    float d;
    float s;
    float r;
};

struct adsrT adsr_vol = {0.25f, 0.25f, 1.0f, 0.01f};
struct adsrT adsr_fil = {1.0f, 0.25f, 1.0f, 0.01f};

typedef enum
{
    attack, decay, sustain, release
} adsr_phaseT;

/* this prototype is required .. others not -  i still do not know what magic arduino is doing */
inline bool ADSR_Process(const struct adsrT *ctrl, float *ctrlSig, adsr_phaseT *phase);

struct filterCoeffT
{
    float aNorm[2] = {0.0f, 0.0f};
    float bNorm[3] = {1.0f, 0.0f, 0.0f};
};

struct filterProcT
{
    struct filterCoeffT *filterCoeff;
    float w[3];
};

struct filterCoeffT filterGlobalC;
struct filterProcT mainFilterL, mainFilterR;

float modulationDepth = 0.0f;
float modulationSpeed = 5.0f;
float modulationPitch = 1.0f;
float pitchBendValue = 0.0f;
float pitchMultiplier = 1.0f;

struct oscillatorT
{
    float **waveForm;
    float *dest;
    uint32_t samplePos;
    uint32_t addVal;
    float *vol;
    float *prc;
    float ctrl;
    float *ctrlSrc;
    float sig;
};

float voiceSink[2];
struct oscillatorT oscPlayer[MAX_POLY_OSC];

uint32_t osc_act = 0;

struct notePlayerT
{
    float lastSample[2];

    float velocity;
    bool active;
    adsr_phaseT phase;
    uint8_t midiNote;

    float control_sign;
    float out_level;

    struct filterCoeffT filterC;
    struct filterProcT filterL;
    struct filterProcT filterR;
    float f_control_sign;
    float f_control_sign_slow;
    adsr_phaseT f_phase;
};

struct notePlayerT voicePlayer[MAX_POLY_VOICE];

uint32_t voc_act = 0;

static float const_null = 0.0f;

void Synth_Init()
{
    randomSeed(34547379);

    /*
     * we do not check if malloc was successful
     * if there is not enough memory left the application will crash
     */
    sine = (float *)malloc(sizeof(float) * WAVEFORM_CNT);
    saw = (float *)malloc(sizeof(float) * WAVEFORM_CNT);
    square = (float *)malloc(sizeof(float) * WAVEFORM_CNT);
    pulse = (float *)malloc(sizeof(float) * WAVEFORM_CNT);
    tri = (float *)malloc(sizeof(float) * WAVEFORM_CNT);
    crappy_noise = (float *)malloc(sizeof(float) * WAVEFORM_CNT);
    silence = (float *)malloc(sizeof(float) * WAVEFORM_CNT);

    /*
     * let us calculate some waveforms
     * - using lookup tables can save a lot of processing power later
     * - but it does consume memory
     */
    for (int i = 0; i < WAVEFORM_CNT; i++)
    {
        float val = (float)sin(i * 2.0 * PI / WAVEFORM_CNT);
        sine[i] = val;
        saw[i] = (2.0f * ((float)i) / ((float)WAVEFORM_CNT)) - 1.0f;
        square[i] = (i > (WAVEFORM_CNT / 2)) ? 1 : -1;
        pulse[i] = (i > (WAVEFORM_CNT / 4)) ? 1 : -1;
        tri[i] = ((i > (WAVEFORM_CNT / 2)) ? (((4.0f * (float)i) / ((float)WAVEFORM_CNT)) - 1.0f) : (3.0f - ((4.0f * (float)i) / ((float)WAVEFORM_CNT)))) - 2.0f;
        crappy_noise[i] = (random(1024) / 512.0f) - 1.0f;
        silence[i] = 0;
    }


    float offset = 0;
    for (int i = 0; i < WAVEFORM_CNT; i++)
    {
        offset += pulse[i];
    }
    offset /= WAVEFORM_CNT;

    for (int i = 0; i < WAVEFORM_CNT; i++)
    {
        pulse[i] -= offset;
    }

    float wav[WAVEFORM_CNT / 4];
    for (int i = 0; i < WAVEFORM_CNT / 4; i++)
    {
        wav[i] = tri[i];
    }

    for (int i = 0; i < WAVEFORM_CNT; i++)
    {
        if (i < (3 * (WAVEFORM_CNT / 4)))
        {
            tri[i] = tri[i + (WAVEFORM_CNT / 4)] ;
        }
        else
        {
            tri[i] = wav[i - 3 * (WAVEFORM_CNT / 4)];
        }
    }

    waveFormLookUp[0] = sine;
    waveFormLookUp[1] = saw;
    waveFormLookUp[2] = square;
    waveFormLookUp[3] = pulse;
    waveFormLookUp[4] = tri;
    waveFormLookUp[5] = crappy_noise;
    waveFormLookUp[6] = silence;

    chCfg.selectedWaveForm =  pulse;

    /*
     * initialize all oscillators
     */
    for (int i = 0; i < MAX_POLY_OSC; i++)
    {
        oscillatorT *osc = &oscPlayer[i];
        osc->waveForm = &silence;
        osc->dest = voiceSink;
        osc->vol = &const_null;
        osc->prc = &const_null;
        osc->ctrlSrc = &const_null;
    }

    /*
     * initialize all voices
     */
    for (int i = 0; i < MAX_POLY_VOICE; i++)
    {
        notePlayerT *voice = &voicePlayer[i];
        voice->active = false;
        voice->lastSample[0] = 0.0f;
        voice->lastSample[1] = 0.0f;
        voice->filterL.filterCoeff = &voice->filterC;
        voice->filterR.filterCoeff = &voice->filterC;
    }

    /*
     * prepare lookup for constants to drive oscillators
     */
    for (int i = 0; i < MIDI_NOTE_CNT; i++)
    {
        float f = ((pow(2.0f, (float)(i - 69) / 12.0f) * 440.0f));
        uint32_t add = (uint32_t)(f * ((float)(1ULL << 32ULL) / ((float)SAMPLE_RATE)));
        midi_note_to_add[i] = add;
    }

    /*
     * assign main filter
     */
    mainFilterL.filterCoeff = &filterGlobalC;
    mainFilterR.filterCoeff = &filterGlobalC;
}

struct filterCoeffT mainFilt;

/*
 * filter calculator:
 * https://www.earlevel.com/main/2013/10/13/biquad-calculator-v2/
 *
 * some filter implementations:
 * https://github.com/ddiakopoulos/MoogLadders/blob/master/src/Filters.h
 *
 * some more information about biquads:
 * https://www.earlevel.com/main/2003/02/28/biquads/
 */

static float filtCutoff = 1.0f;
static float filtReso = 0.5f;
static float soundFiltReso = 0.5f;
static float soundNoiseLevel = 0.0f;

/*
 * calculate coefficients of the 2nd order IIR filter
 */
inline void Filter_Calculate(float c, float reso, struct filterCoeffT *const  filterC)
{
    float *aNorm = filterC->aNorm;
    float *bNorm = filterC->bNorm;

    float Q = reso;
    float  cosOmega, omega, sinOmega, alpha, a[3], b[3];

    /*
     * change curve of cutoff a bit
     * maybe also log or exp function could be used
     */
    c = c * c * c;

    if (c >= 1.0f)
    {
        omega = 1.0f;
    }
    else if (c < 0.0025f)
    {
        omega = 0.0025f;
    }
    else
    {
        omega = c;
    }

    /*
     * use lookup here to get quicker results
     */
    cosOmega = sine[WAVEFORM_I((uint32_t)((float)((1ULL << 31) - 1) * omega + (float)((1ULL << 30) - 1)))];
    sinOmega = sine[WAVEFORM_I((uint32_t)((float)((1ULL << 31) - 1) * omega))];

    alpha = sinOmega / (2.0 * Q);
    b[0] = (1 - cosOmega) / 2;
    b[1] = 1 - cosOmega;
    b[2] = b[0];
    a[0] = 1 + alpha;
    a[1] = -2 * cosOmega;
    a[2] = 1 - alpha;

    // Normalize filter coefficients
    float factor = 1.0f / a[0];

    aNorm[0] = a[1] * factor;
    aNorm[1] = a[2] * factor;

    bNorm[0] = b[0] * factor;
    bNorm[1] = b[1] * factor;
    bNorm[2] = b[2] * factor;
}

inline void Filter_Process(float *const signal, struct filterProcT *const filterP)
{
    const float out = filterP->filterCoeff->bNorm[0] * (*signal) + filterP->w[0];
    filterP->w[0] = filterP->filterCoeff->bNorm[1] * (*signal) - filterP->filterCoeff->aNorm[0] * out + filterP->w[1];
    filterP->w[1] = filterP->filterCoeff->bNorm[2] * (*signal) - filterP->filterCoeff->aNorm[1] * out;
    *signal = out;
}

/*
 * very bad and simple implementation of ADSR
 * - but it works for the start
 */
inline bool ADSR_Process(const struct adsrT *ctrl, float *ctrlSig, adsr_phaseT *phase)
{
    switch (*phase)
    {
    case attack:
        *ctrlSig += ctrl->a;
        if (*ctrlSig > 1.0f)
        {
            *ctrlSig = 1.0f;
            *phase = decay;
        }
        break;
    case decay:
        *ctrlSig -= ctrl->d;
        if (*ctrlSig < ctrl->s)
        {
            *ctrlSig = ctrl->s;
            *phase = sustain;
        }
        break;
    case sustain:
        break;
    case release:
        *ctrlSig -= ctrl->r;
        if (*ctrlSig < 0.0f)
        {
            *ctrlSig = 0.0f;
            //voice->active = false;
            return false;
        }
    }
    return true;
}

void Voice_Off(uint32_t i)
{
    notePlayerT *voice = &voicePlayer[i];
    for (int f = 0; f < MAX_POLY_OSC; f++)
    {
        oscillatorT *osc = &oscPlayer[f];
        if (osc->dest == voice->lastSample)
        {
            osc->dest = voiceSink;
            osc_act -= 1;
            osc->vol = &const_null;
            clk_l += osc->sig;
        }
    }
    if (voice->active)
    {
        voice->active = false;
        voc_act -= 1;
    }
}

inline
float SineNorm(float alpha_div2pi)
{
    uint32_t index = ((uint32_t)(alpha_div2pi * ((float)WAVEFORM_CNT))) % WAVEFORM_CNT;
    return sine[index];
}

inline
float GetModulation(void)
{
    float modSpeed = modulationSpeed;
    return modulationDepth * modulationPitch * (SineNorm((modSpeed * ((float)millis()) / 1000.0f)));
}

static float out_l, out_r;
static uint32_t count = 0;

//[[gnu::noinline, gnu::optimize ("fast-math")]]
inline void Synth_Process(float *left, float *right)
{
    chCfg.percSig *= chCfg.percRel;
    Synth_ProcessSelectCnt();

    /*
     * generator simulation, rotate all wheels
     */
    out_l = 0;
    out_r = 0;

    /* counter required to optimize processing */
    count += 1;

    /*
     * destination for unused oscillators
     */
    voiceSink[0] = 0;
    voiceSink[1] = 0;

    /*
     * update pitch bending / modulation
     */
    if (count % 64 == 0)
    {
        float pitchVar = pitchBendValue + GetModulation();
        //static float lastPitchVar = 0;
        pitchMultiplier = pow(2.0f, pitchVar / 12.0f);
    }

    /*
     * oscillator processing -> mix to voice
     */
    for (int i = 0; i < MAX_POLY_OSC; i++)
    {
        oscillatorT *osc = &oscPlayer[i];
        {
            osc->samplePos += (uint32_t)(pitchMultiplier * ((float)osc->addVal));
            osc->sig = (*osc->waveForm)[WAVEFORM_I(osc->samplePos)];
            osc->sig *= *(osc->vol);
            osc->ctrl *= *(osc->prc);
            osc->sig *= *osc->ctrlSrc;
            out_l += osc->sig;
            out_r += osc->sig;
        }
    }

    clk_l *= 0.9995;

    out_l += clk_l;
    out_r += clk_l;

    /*
     * process main filter
     */
    Filter_Process(&out_l, &mainFilterL);
    Filter_Process(&out_r, &mainFilterR);

    /*
     * reduce level a bit to avoid distortion
     */
    out_l *= 0.1f * 0.25f;
    out_r *= 0.1f * 0.25f;

    /*
     * finally output our samples
     */
    *left = out_l;
    *right = out_r;
}

struct oscillatorT *getFreeOsc()
{
    for (int i = 0; i < MAX_POLY_OSC ; i++)
    {
        if (oscPlayer[i].dest == voiceSink)
        {
            return &oscPlayer[i];
        }
    }
    return NULL;
}

struct notePlayerT *getFreeVoice()
{
    for (int i = 0; i < MAX_POLY_VOICE ; i++)
    {
        if (voicePlayer[i].active == false)
        {
            return &voicePlayer[i];
        }
    }
    return NULL;
}

inline void Filter_Reset(struct filterProcT *filter)
{
    filter->w[0] = 0.0f;
    filter->w[1] = 0.0f;
    filter->w[2] = 0.0f;
}

inline void Synth_NoteOn(uint8_t ch, uint8_t note, float vel)
{
    struct notePlayerT *voice = getFreeVoice();
    struct oscillatorT *osc = getFreeOsc();

    if (voc_act == 0)
    {
        chCfg.percSig = 4.0f;
    }

    /*
     * No free voice found, return otherwise crash xD
     */
    if ((voice == NULL) || (osc == NULL))
    {
        //Serial.printf("voc: %d, osc: %d\n", voc_act, osc_act);
        return ;
    }

    voice->midiNote = note;
#ifdef MIDI_USE_CONST_VELOCITY
    voice->velocity = 1.0f;
#else
    voice->velocity = vel;
#endif
    voice->lastSample[0] = 0.0f;
    voice->lastSample[1] = 0.0f;
    voice->control_sign = 0.0f;


    if (adsr_fil.a < adsr_fil.s)
    {
        adsr_fil.a = adsr_fil.s;
    }
    voice->f_phase = decay;
    voice->f_control_sign = adsr_fil.a;
    voice->f_control_sign_slow = adsr_fil.a;
    voice->phase = attack;

    voice->active = true;
    voc_act += 1;

    for (int i = 0; i < 9; i++)
    {
        osc = getFreeOsc();
        if (osc == NULL)
        {
            Serial.printf("voc: %d, osc: %d\n", voc_act, osc_act);
            break ;
        }

        int oNote = note + chCfg.dbOffset[i] - 12;
        if ((oNote > 0) && (oNote < 128))
        {
            osc->addVal = midi_note_to_add[oNote];
            //osc->samplePos = (uint32_t)random(1 << 31); /* otherwise it sounds ... bad!? */
            osc->samplePos = 0;
            osc->waveForm = &chCfg.selectedWaveForm;
            osc->dest = voice->lastSample; /* required to attach to voice */
            osc->vol = &chCfg.drawbar[i];
            if (i == chCfg.percNote)
            {
                osc->ctrlSrc = &chCfg.percSig;
                osc->prc = &chCfg.percRel;
            }
            else
            {
                osc->ctrl = 1.0f;
                osc->ctrlSrc = &osc->ctrl;
                osc->prc = &chCfg.perc[i];
            }
            osc_act += 1;
        }
    }

    /*
     * trying to avoid audible suprises
     */
    Filter_Reset(&voice->filterL);
    Filter_Reset(&voice->filterR);
    Filter_Process(&voice->lastSample[0], &voice->filterL);
    Filter_Process(&voice->lastSample[0], &voice->filterL);
    Filter_Process(&voice->lastSample[0], &voice->filterL);

    Filter_Process(&voice->lastSample[1], &voice->filterR);
    Filter_Process(&voice->lastSample[1], &voice->filterR);
    Filter_Process(&voice->lastSample[1], &voice->filterR);
}

inline void Synth_NoteOff(uint8_t ch, uint8_t note)
{
    for (int i = 0; i < MAX_POLY_VOICE ; i++)
    {
        if ((voicePlayer[i].active) && (voicePlayer[i].midiNote == note))
        {
            Voice_Off(i);
        }
    }
}

void Synth_ModulationWheel(uint8_t ch, float value)
{
    modulationDepth = value;
}

void Synth_ModulationSpeed(uint8_t ch, float value)
{
    modulationSpeed = value * 10;
    //Status_ValueChangedFloat("ModulationSpeed", modulationSpeed);
}

void Synth_ModulationPitch(uint8_t ch, float value)
{
    modulationPitch = value * 5;
    //Status_ValueChangedFloat("ModulationDepth", modulationPitch);
}

void Synth_PitchBend(uint8_t ch, float bend)
{
    pitchBendValue = bend;
    Serial.printf("pitchBendValue: %0.3f\n", pitchBendValue);
}

void Synth_SetFader(uint8_t slider, float value)
{
    if (slider < 9)
    {
        chCfg.drawbar[slider] = value;
        Status_ValueChangedFloatArr("drawbar", value, slider);
    }
}

void Synth_SetPercRel(uint8_t unused, float value)
{
    if (chCfg.sel_bar < 9)
    {
        if (value == 1.0f)
        {
            chCfg.perc[chCfg.sel_bar] = 1.0f;
        }
        else
        {
            chCfg.perc[chCfg.sel_bar] = 1.0f - (0.00001 * pow(100, 1.0f - value));// pow(2, value * 12);
        }
        Status_ValueChangedFloatArr("PercRel", value, chCfg.sel_bar);
    }
    else
    {
        chCfg.percRel = 1.0f - (0.00001 * pow(100, 1.0f - value));// pow(2, value * 12);
        Status_ValueChangedFloat("PercRel", value);
    }
}

#if 0
static uint32_t sel_cnt = 0;
#endif

void Synth_ProcessSelectCnt()
{
#if 0
    if (chCfg.sel_bar < 9)
    {
        sel_cnt++;
        if (sel_cnt > (SAMPLE_RATE * 3))
        {
            if (chCfg.percNote != chCfg.sel_bar)
            {
                chCfg.percNote = chCfg.sel_bar;
                Serial.printf("percNote =  %d\n", chCfg.percNote);
            }
        }
    }
    else
    {
        sel_cnt = 0;
    }
#endif
}

void SynthSelect(uint8_t bar, float value)
{
    if (value > 0)
    {
        if (bar == chCfg.percNote)
        {
            chCfg.sel_bar = 0xFF;
        }
        else
        {
            chCfg.sel_bar = bar;
        }
        Status_ValueChangedInt("PercDbSel", chCfg.sel_bar);
    }
}

void SynthSelectPerc(uint8_t unusced, float value)
{
    if (value > 0)
    {
        chCfg.percNote = chCfg.sel_bar;
        chCfg.sel_bar = 0xFF; /* to keep assigned release control to perc note */
        Status_ValueChangedInt("PercNote", chCfg.sel_bar);
    }
}

void Synth_SetParam(uint8_t slider, float value)
{
    switch (slider)
    {
    case SYNTH_PARAM_VEL_ENV_ATTACK:
        adsr_vol.a = (0.00005 * pow(5000, 1.0f - value));
        Status_ValueChangedFloat("voice volume attack", adsr_vol.a);
        break;
    case SYNTH_PARAM_VEL_ENV_DECAY:
        adsr_vol.d = (0.00005 * pow(5000, 1.0f - value));
        Status_ValueChangedFloat("voice volume decay", adsr_vol.d);
        break;
    case SYNTH_PARAM_VEL_ENV_SUSTAIN:
        adsr_vol.s = (0.01 * pow(100, value));
        Status_ValueChangedFloat("voice volume sustain", adsr_vol.s);
        break;
    case SYNTH_PARAM_VEL_ENV_RELEASE:
        adsr_vol.r = (0.0001 * pow(100, 1.0f - value));
        Status_ValueChangedFloat("voice volume release", adsr_vol.r);
        break;

    case SYNTH_PARAM_FIL_ENV_ATTACK:
#if 0
        adsr_fil.a = (0.00005 * pow(5000, 1.0f - value));
#else
        adsr_fil.a = value;
#endif
        Status_ValueChangedFloat("voice filter attack", adsr_fil.a);
        break;
    case SYNTH_PARAM_FIL_ENV_DECAY:
        adsr_fil.d = (0.00005 * pow(5000, 1.0f - value));
        Status_ValueChangedFloat("voice filter decay", adsr_fil.d);
        break;
    case SYNTH_PARAM_FIL_ENV_SUSTAIN:
        adsr_fil.s = value;
        Status_ValueChangedFloat("voice filter sustain", adsr_fil.s);
        break;
    case SYNTH_PARAM_FIL_ENV_RELEASE:
        adsr_fil.r = (0.0001 * pow(100, 1.0f - value));
        Status_ValueChangedFloat("voice filter release", adsr_fil.r);
        break;

    case SYNTH_PARAM_WAVEFORM_1:
        {
            uint8_t selWaveForm = (value) * (WAVEFORM_TYPE_COUNT);
            chCfg.selectedWaveForm = waveFormLookUp[selWaveForm];
            Status_ValueChangedInt("selWaveForm", selWaveForm);
#ifdef SPI_DISP_ENABLED
            Display_DisplayWaveform(chCfg.selectedWaveForm, WAVEFORM_CNT);
#endif
        }
        break;

    case SYNTH_PARAM_MAIN_FILT_CUTOFF:
        filtCutoff = value;
        Status_ValueChangedFloat("main filter cutoff", filtCutoff);
        Filter_Calculate(filtCutoff, filtReso, &filterGlobalC);
        break;
    case SYNTH_PARAM_MAIN_FILT_RESO:
        filtReso =  0.5f + 10 * value * value * value; /* min q is 0.5 here */
        Status_ValueChangedFloat("main filter reso", filtReso);
        Filter_Calculate(filtCutoff, filtReso, &filterGlobalC);
        break;

    case SYNTH_PARAM_VOICE_FILT_RESO:
        soundFiltReso = 0.5f + 10 * value * value * value; /* min q is 0.5 here */
        Status_ValueChangedFloat("voice filter reso", soundFiltReso);
        break;

    case SYNTH_PARAM_VOICE_NOISE_LEVEL:
        soundNoiseLevel = value;
        Status_ValueChangedFloat("voice noise level", soundNoiseLevel);
        break;

    default:
        /* not connected */
        break;
    }
}

