#pragma once

#include <cstdint>

enum class MsgType : uint8_t
{
  DATA = 33,
  CMD = 64,
  DEFAULT_CMD = 0
};
