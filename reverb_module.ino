/*
 * reverb stuff
 *
 * This is a port of the public code from YetAnotherElectronicsChannel
 * Based on main.c I've created this module
 *
 * src: https://github.com/YetAnotherElectronicsChannel/STM32_DSP_Reverb/blob/master/code/Src/main.c
 *
 * The explanation of the original module can be found here: https://youtu.be/nRLXNmLmHqM
 *
 * Changes to its original:
 * - ported into an arduino project
 * - optimized for buffer processing
 * - added interface to set the level
 *
 * Big thanks to "YetAnotherElectronicsChannel" allowing me to convert the reverb example to integrate it into the synthesizer project!
 */

/**
 * @file reverb_module.ino
 * @see [#8] Reverb - Audio DSP On STM32 (24 Bit / 96 kHz) - https://youtu.be/nRLXNmLmHqM
 */


#ifdef __CDT_PARSER__
#include <cdt.h>
#endif


#define l_CB0 3460
#define l_CB1 2988
#define l_CB2 3882
#define l_CB3 4312
#define l_AP0 480
#define l_AP1 161
#define l_AP2 46

//define wet 0.0 <-> 1.0
//float wet = 1.0f;
//define time delay 0.0 <-> 1.0 (max)
float rev_time = 1.0f;
float rev_level = 0.0f;

//define pointer limits = delay time
int cf0_lim, cf1_lim, cf2_lim, cf3_lim, ap0_lim, ap1_lim, ap2_lim;

inline void Do_Comb0(float *inSample, float *outSample, int buffLen)
{
    for (int n = 0; n < buffLen; n++)
    {
        static float cfbuf0[l_CB0];
        static int cf0_p = 0;
        static float cf0_g = 0.805f;

        float readback = cfbuf0[cf0_p];
        float newV = readback * cf0_g + inSample[n];
        cfbuf0[cf0_p] = newV;
        cf0_p++;
        if (cf0_p == cf0_lim)
        {
            cf0_p = 0;
        }
        outSample[n] += readback;
    }
}

inline void Do_Comb1(float *inSample, float *outSample, int buffLen)
{
    for (int n = 0; n < buffLen; n++)
    {
        static float cfbuf1[l_CB1];
        static int cf1_p = 0;
        static float cf1_g = 0.827f;

        float readback = cfbuf1[cf1_p];
        float newV = readback * cf1_g + inSample[n];
        cfbuf1[cf1_p] = newV;
        cf1_p++;
        if (cf1_p == cf1_lim)
        {
            cf1_p = 0;
        }
        outSample[n] += readback;
    }
}

inline void Do_Comb2(float *inSample, float *outSample, int buffLen)
{
    for (int n = 0; n < buffLen; n++)
    {
        static float cfbuf2[l_CB2];
        static int cf2_p = 0;
        static float cf2_g = 0.783f;

        float readback = cfbuf2[cf2_p];
        float newV = readback * cf2_g + inSample[n];
        cfbuf2[cf2_p] = newV;
        cf2_p++;
        if (cf2_p == cf2_lim)
        {
            cf2_p = 0;
        }
        outSample[n] += readback;
    }
}

inline void Do_Comb3(float *inSample, float *outSample, int buffLen)
{
    for (int n = 0; n < buffLen; n++)
    {
        static float cfbuf3[l_CB3];
        static int cf3_p = 0;
        static float cf3_g = 0.764f;

        float readback = cfbuf3[cf3_p];
        float newV = readback * cf3_g + inSample[n];
        cfbuf3[cf3_p] = newV;
        cf3_p++;
        if (cf3_p == cf3_lim)
        {
            cf3_p = 0;
        }
        outSample[n] += readback;
    }
}


inline void Do_Allpass0(float *inSample, int buffLen)
{
    for (int n = 0; n < buffLen; n++)
    {
        static float apbuf0[l_AP0];
        static int ap0_p = 0;
        static float ap0_g = 0.7f;

        float readback = apbuf0[ap0_p];
        readback += (-ap0_g) * inSample[n];
        float newV = readback * ap0_g + inSample[n];
        apbuf0[ap0_p] = newV;
        ap0_p++;
        if (ap0_p == ap0_lim)
        {
            ap0_p = 0;
        }
        inSample[n] = readback;
    }
}

inline void Do_Allpass1(float *inSample, int buffLen)
{
    for (int n = 0; n < buffLen; n++)
    {
        static float apbuf1[l_AP1];
        static int ap1_p = 0;
        static float ap1_g = 0.7f;

        float readback = apbuf1[ap1_p];
        readback += (-ap1_g) * inSample[n];
        float newV = readback * ap1_g + inSample[n];
        apbuf1[ap1_p] = newV;
        ap1_p++;
        if (ap1_p == ap1_lim)
        {
            ap1_p = 0;
        }
        inSample[n] = readback;
    }
}

inline void Do_Allpass2(float *inSample, int buffLen)
{
    for (int n = 0; n < buffLen; n++)
    {
        static float apbuf2[l_AP2];
        static int ap2_p = 0;
        static float ap2_g = 0.7f;

        float readback = apbuf2[ap2_p];
        readback += (-ap2_g) * inSample[n];
        float newV = readback * ap2_g + inSample[n];
        apbuf2[ap2_p] = newV;
        ap2_p++;
        if (ap2_p == ap2_lim)
        {
            ap2_p = 0;
        }
        inSample[n] = readback;
    }
}

inline void Reverb_Process(float *signal_l, int buffLen)
{
    float inSample[buffLen];
    for (int n = 0; n < buffLen; n++)
    {
        /* create mono sample */
        inSample[n] = signal_l[n]; /* it may cause unwanted audible effects */
    }
    float newsample[buffLen];
    memset(newsample, 0, sizeof(newsample));
    Do_Comb0(inSample, newsample, buffLen);
    Do_Comb1(inSample, newsample, buffLen);
    Do_Comb2(inSample, newsample, buffLen);
    Do_Comb3(inSample, newsample, buffLen);
    for (int n = 0; n < buffLen; n++)
    {
        newsample[n] *= 0.25f;
    }

    Do_Allpass0(newsample, buffLen);
    Do_Allpass1(newsample, buffLen);
    Do_Allpass2(newsample, buffLen);

    /* apply reverb level */
    for (int n = 0; n < buffLen; n++)
    {
        newsample[n] *= rev_level;
        signal_l[n] += newsample[n];
    }
}

inline void Reverb_Setup(void)
{
    cf0_lim = (int)(rev_time * l_CB0);
    cf1_lim = (int)(rev_time * l_CB1);
    cf2_lim = (int)(rev_time * l_CB2);
    cf3_lim = (int)(rev_time * l_CB3);
    ap0_lim = (int)(rev_time * l_AP0);
    ap1_lim = (int)(rev_time * l_AP1);
    ap2_lim = (int)(rev_time * l_AP2);
}

inline void Reverb_SetLevel(uint8_t not_used, float value)
{
    rev_level = value;
    Status_ValueChangedFloat("ReverbLevel", rev_level);
}
