#ifndef GLIDER_C01_PROTO_H
#define GLIDER_C01_PROTO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define GLIDER_C01_MAX_FRAME_SIZE 64U

typedef enum
{
  GLIDER_C01_FUNC_TX_REMOTE = 0x6AU,
  GLIDER_C01_FUNC_RX_REMOTE_ACK = 0x8AU,
  GLIDER_C01_FUNC_RX_REMOTE = 0x8BU,
  GLIDER_C01_FUNC_TX_POSITION = 0x6CU,
  GLIDER_C01_FUNC_RX_POSITION = 0x8CU,
  GLIDER_C01_FUNC_TX_TRIM = 0x6DU,
  GLIDER_C01_FUNC_RX_TRIM = 0x8DU,
  GLIDER_C01_FUNC_TX_MULTI = 0x6EU,
  GLIDER_C01_FUNC_RX_MULTI = 0x8EU
} GliderC01FunctionCode;

typedef struct
{
  uint8_t address;
  uint8_t function;
  uint8_t length;
  uint8_t status;
  uint8_t payload[GLIDER_C01_MAX_FRAME_SIZE - 5U];
  size_t payload_len;
  uint8_t checksum;
  bool checksum_valid;
} GliderC01Frame;

uint8_t GliderC01_Checksum(const uint8_t *data, size_t len);
bool GliderC01_VerifyFrame(const uint8_t *frame, size_t frame_len);
bool GliderC01_TryParseFrame(const uint8_t *frame, size_t frame_len, GliderC01Frame *out);

bool GliderC01_BuildRemoteCommand(uint8_t address, uint8_t data, uint8_t *out, size_t out_capacity, size_t *out_len);
bool GliderC01_BuildPositionCommand(uint8_t address, uint8_t *out, size_t out_capacity, size_t *out_len);

#endif
