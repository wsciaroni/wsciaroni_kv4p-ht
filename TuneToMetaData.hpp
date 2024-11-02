#pragma once

#include "Arduino.h"

/// @brief
// 145.450144.850061
// 7 chars for tx, 7 chars for rx, 2 chars for tone, 2 char for squelch (18 bytes total for params)
struct TuneToMetaData
{
    uint8_t paramsStr[18];

    TuneToMetaData(TuneToMetaData *other)
    {
        strncpy((char *)paramsStr, (char *)other->paramsStr, 18);
    }

    String getParamsString() const
    {
        String paramsStrRtrn(paramsStr, 18);
        return paramsStrRtrn;
    }

    float getTxFreq() const
    {
        return getParamsString().substring(0, 7).toFloat();
    }

    float getRxFreq() const
    {
        return getParamsString().substring(7, 14).toFloat();
    }

    int getToneInt() const
    {
        return getParamsString().substring(14, 16).toInt();
    }

    int getSquelchInt() const
    {
        return getParamsString().substring(16, 17).toInt();
    }

    void log() const
    {
        Serial.println("PARAMS: " + getParamsString().substring(0, 16) + " freqTxFloat: " + String(getTxFreq()) + " freqRxFloat: " + String(getRxFreq()) + " toneInt: " + String(getToneInt()));
    }
};
