#include "glider_c01_proto.h"

#include <string.h>

uint8_t GliderC01_Checksum(const uint8_t *data, size_t len)
{
  uint32_t sum = 0U;
  size_t i = 0U;

  if (data == NULL)
  {
    return 0U;
  }

  for (i = 0U; i < len; i++)
  {
    sum += data[i];
  }

  return (uint8_t)(sum & 0xFFU);
}

bool GliderC01_VerifyFrame(const uint8_t *frame, size_t frame_len)
{
  if (frame == NULL || frame_len < 5U)
  {
    return false;
  }

  if (frame[2] != (uint8_t)frame_len)
  {
    return false;
  }

  return GliderC01_Checksum(frame, frame_len - 1U) == frame[frame_len - 1U];
}

bool GliderC01_TryParseFrame(const uint8_t *frame, size_t frame_len, GliderC01Frame *out)
{
  size_t payload_len = 0U;

  if (frame == NULL || out == NULL || frame_len < 5U || frame_len > GLIDER_C01_MAX_FRAME_SIZE)
  {
    return false;
  }

  memset(out, 0, sizeof(*out));
  out->address = frame[0];
  out->function = frame[1];
  out->length = frame[2];
  out->status = frame[3];
  out->checksum = frame[frame_len - 1U];
  out->checksum_valid = GliderC01_VerifyFrame(frame, frame_len);

  if (out->length != (uint8_t)frame_len)
  {
    return false;
  }

  payload_len = frame_len - 5U;
  out->payload_len = payload_len;
  if (payload_len > 0U)
  {
    memcpy(out->payload, &frame[4], payload_len);
  }

  return true;
}

bool GliderC01_BuildRemoteCommand(uint8_t address, uint8_t data, uint8_t *out, size_t out_capacity, size_t *out_len)
{
  uint8_t frame[7];

  if (out == NULL || out_len == NULL || out_capacity < sizeof(frame))
  {
    return false;
  }

  frame[0] = address;
  frame[1] = (uint8_t)GLIDER_C01_FUNC_TX_REMOTE;
  frame[2] = 7U;
  frame[3] = 0x01U;
  frame[4] = 0xFFU;
  frame[5] = data;
  frame[6] = GliderC01_Checksum(frame, 6U);

  memcpy(out, frame, sizeof(frame));
  *out_len = sizeof(frame);
  return true;
}

bool GliderC01_BuildPositionCommand(uint8_t address, uint8_t *out, size_t out_capacity, size_t *out_len)
{
  uint8_t frame[7];

  if (out == NULL || out_len == NULL || out_capacity < sizeof(frame))
  {
    return false;
  }

  frame[0] = address;
  frame[1] = (uint8_t)GLIDER_C01_FUNC_TX_POSITION;
  frame[2] = 7U;
  frame[3] = 0x01U;
  frame[4] = 0xFFU;
  frame[5] = 0x01U;
  frame[6] = GliderC01_Checksum(frame, 6U);

  memcpy(out, frame, sizeof(frame));
  *out_len = sizeof(frame);
  return true;
}
