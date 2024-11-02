#pragma once

#include "Arduino.h"

struct FiltersMetaData
{
    uint8_t paramsStr[3];

    FiltersMetaData(FiltersMetaData *other)
    {
        strncpy((char *)paramsStr, (char *)other->paramsStr, 3);
    }

    String getParamsString() const
    {
        String paramsStrRtrn(paramsStr, 3);
        return paramsStrRtrn;
    }

    bool getEmphasis() const
    {
        return (getParamsString().charAt(0) == '1');
    }

    bool getHighpass() const
    {
        return (getParamsString().charAt(1) == '1');
    }

    bool getLowpass() const
    {
        return (getParamsString().charAt(2) == '1');
    }
};
