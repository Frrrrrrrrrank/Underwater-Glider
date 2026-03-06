#ifndef GLIDER_RDSS_CODEC_H
#define GLIDER_RDSS_CODEC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum
{
  GLIDER_RDSS_FRAME_UNKNOWN = 0,
  GLIDER_RDSS_FRAME_PLAIN_TEXT,
  GLIDER_RDSS_FRAME_NMEA_TEXT,
  GLIDER_RDSS_FRAME_AT_RESPONSE,
  GLIDER_RDSS_FRAME_HEX_BINARY
} GliderRdssFrameType;

typedef struct
{
  GliderRdssFrameType type;
  bool has_checksum;
  bool checksum_valid;
  uint8_t checksum_rx;
  uint8_t checksum_calc;
  char sentence[24];
  char command[24];
  char text[192];
  uint8_t payload[128];
  size_t payload_len;
} GliderRdssDecodedFrame;

bool GliderRdssCodec_TryDecodeLine(const char *line, GliderRdssDecodedFrame *out_frame);

#endif
