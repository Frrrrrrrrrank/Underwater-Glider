#include "glider_rdss_codec.h"

#include <ctype.h>
#include <string.h>

static size_t bounded_strnlen(const char *s, size_t max_len)
{
  size_t n = 0;

  while (n < max_len && s[n] != '\0')
  {
    n++;
  }

  return n;
}

static void copy_token(char *dst, size_t dst_size, const char *src, size_t src_len)
{
  size_t copy_len = 0;

  if (dst == NULL || dst_size == 0U)
  {
    return;
  }

  if (src == NULL)
  {
    dst[0] = '\0';
    return;
  }

  copy_len = src_len;
  if (copy_len >= (dst_size - 1U))
  {
    copy_len = dst_size - 1U;
  }

  if (copy_len > 0U)
  {
    memcpy(dst, src, copy_len);
  }
  dst[copy_len] = '\0';
}

static uint8_t calc_xor_checksum(const char *data, size_t len)
{
  uint8_t checksum = 0U;
  size_t i = 0;

  for (i = 0; i < len; i++)
  {
    checksum ^= (uint8_t)data[i];
  }

  return checksum;
}

static bool parse_hex_byte(const char hi, const char lo, uint8_t *out)
{
  int value_hi = 0;
  int value_lo = 0;

  if (!isxdigit((unsigned char)hi) || !isxdigit((unsigned char)lo))
  {
    return false;
  }

  value_hi = isdigit((unsigned char)hi) ? (hi - '0') : (tolower((unsigned char)hi) - 'a' + 10);
  value_lo = isdigit((unsigned char)lo) ? (lo - '0') : (tolower((unsigned char)lo) - 'a' + 10);

  *out = (uint8_t)((value_hi << 4) | value_lo);
  return true;
}

static bool parse_nmea_like_frame(const char *line, size_t line_len, GliderRdssDecodedFrame *out_frame)
{
  const char *asterisk = NULL;
  size_t body_len = 0;
  size_t sentence_len = 0;
  const char *comma = NULL;

  if (line_len < 2U || line[0] != '$')
  {
    return false;
  }

  asterisk = strchr(line, '*');
  body_len = line_len - 1U;

  if (asterisk != NULL)
  {
    size_t cs_index = (size_t)(asterisk - line) + 1U;
    if (cs_index + 2U != line_len)
    {
      return false;
    }

    if (!parse_hex_byte(line[cs_index], line[cs_index + 1U], &out_frame->checksum_rx))
    {
      return false;
    }

    out_frame->has_checksum = true;
    body_len = (size_t)(asterisk - line) - 1U;
  }

  if (body_len == 0U)
  {
    return false;
  }

  out_frame->checksum_calc = calc_xor_checksum(&line[1], body_len);
  out_frame->checksum_valid = (!out_frame->has_checksum) || (out_frame->checksum_rx == out_frame->checksum_calc);

  copy_token(out_frame->text, sizeof(out_frame->text), line, line_len);
  copy_token((char *)out_frame->payload, sizeof(out_frame->payload), &line[1], body_len);
  out_frame->payload_len = body_len;

  comma = memchr(&line[1], ',', body_len);
  if (comma == NULL)
  {
    sentence_len = body_len;
  }
  else
  {
    sentence_len = (size_t)(comma - &line[1]);
  }
  copy_token(out_frame->sentence, sizeof(out_frame->sentence), &line[1], sentence_len);

  out_frame->type = GLIDER_RDSS_FRAME_NMEA_TEXT;
  return true;
}

static bool parse_hex_line(const char *line, size_t line_len, GliderRdssDecodedFrame *out_frame)
{
  size_t i = 0;
  int pending_nibble = -1;

  if (line_len < 4U || strncmp(line, "HEX:", 4) != 0)
  {
    return false;
  }

  for (i = 4U; i < line_len; i++)
  {
    unsigned char ch = (unsigned char)line[i];
    int nibble = 0;

    if (ch == ' ' || ch == '\t' || ch == ',' || ch == '-' || ch == ':')
    {
      continue;
    }

    if ((ch == 'x' || ch == 'X') && i > 0U && line[i - 1U] == '0')
    {
      continue;
    }

    if (!isxdigit(ch))
    {
      return false;
    }

    nibble = isdigit(ch) ? (ch - '0') : (tolower(ch) - 'a' + 10);
    if (pending_nibble < 0)
    {
      pending_nibble = nibble;
      continue;
    }

    if (out_frame->payload_len >= sizeof(out_frame->payload))
    {
      return false;
    }

    out_frame->payload[out_frame->payload_len++] = (uint8_t)((pending_nibble << 4) | nibble);
    pending_nibble = -1;
  }

  if (pending_nibble >= 0)
  {
    return false;
  }

  out_frame->type = GLIDER_RDSS_FRAME_HEX_BINARY;
  copy_token(out_frame->text, sizeof(out_frame->text), line, line_len);
  return out_frame->payload_len > 0U;
}

static bool parse_at_response(const char *line, size_t line_len, GliderRdssDecodedFrame *out_frame)
{
  const char *plus = NULL;
  const char *sep = NULL;

  if (line_len == 0U)
  {
    return false;
  }

  copy_token(out_frame->text, sizeof(out_frame->text), line, line_len);

  if (strncmp(line, "AT", 2) == 0 || strncmp(line, "OK", 2) == 0 || strncmp(line, "ERROR", 5) == 0)
  {
    out_frame->type = GLIDER_RDSS_FRAME_AT_RESPONSE;
    copy_token(out_frame->command, sizeof(out_frame->command), line, line_len);
    return true;
  }

  plus = strchr(line, '+');
  if (plus != NULL)
  {
    sep = strchr(plus, ':');
    if (sep == NULL)
    {
      sep = strchr(plus, ',');
    }
    if (sep == NULL)
    {
      sep = line + line_len;
    }

    copy_token(out_frame->command, sizeof(out_frame->command), plus, (size_t)(sep - plus));
    out_frame->type = GLIDER_RDSS_FRAME_AT_RESPONSE;
    return true;
  }

  return false;
}

bool GliderRdssCodec_TryDecodeLine(const char *line, GliderRdssDecodedFrame *out_frame)
{
  size_t line_len = 0;
  const char *start = NULL;
  const char *end = NULL;
  char normalized_line[192];
  size_t normalized_len = 0;

  if (line == NULL || out_frame == NULL)
  {
    return false;
  }

  memset(out_frame, 0, sizeof(*out_frame));
  line_len = bounded_strnlen(line, sizeof(normalized_line) - 1U);

  if (line_len == 0)
  {
    return false;
  }

  start = line;
  end = line + line_len;

  while (start < end && isspace((unsigned char)*start))
  {
    start++;
  }

  while (end > start && isspace((unsigned char)*(end - 1)))
  {
    end--;
  }

  normalized_len = (size_t)(end - start);
  if (normalized_len == 0U)
  {
    return false;
  }

  copy_token(normalized_line, sizeof(normalized_line), start, normalized_len);

  if (parse_nmea_like_frame(normalized_line, normalized_len, out_frame))
  {
    return true;
  }

  if (parse_hex_line(normalized_line, normalized_len, out_frame))
  {
    return true;
  }

  if (parse_at_response(normalized_line, normalized_len, out_frame))
  {
    return true;
  }

  out_frame->type = GLIDER_RDSS_FRAME_UNKNOWN;
  copy_token(out_frame->text, sizeof(out_frame->text), normalized_line, normalized_len);
  return true;
}
