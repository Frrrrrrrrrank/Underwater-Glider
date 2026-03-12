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

static bool parse_uint_field(const char *src, size_t src_len, uint32_t *out)
{
  size_t i = 0U;
  uint32_t value = 0U;

  if (src == NULL || out == NULL || src_len == 0U)
  {
    return false;
  }

  for (i = 0U; i < src_len; i++)
  {
    if (!isdigit((unsigned char)src[i]))
    {
      return false;
    }

    value = (value * 10U) + (uint32_t)(src[i] - '0');
  }

  *out = value;
  return true;
}

static bool parse_decimal_x100(const char *src, size_t src_len, int32_t *out)
{
  size_t i = 0U;
  int32_t sign = 1;
  int32_t int_part = 0;
  int32_t frac_part = 0;
  int32_t frac_scale = 0;
  bool seen_dot = false;
  bool seen_digit = false;

  if (src == NULL || out == NULL || src_len == 0U)
  {
    return false;
  }

  if (src[0] == '-')
  {
    sign = -1;
    i = 1U;
  }
  else if (src[0] == '+')
  {
    i = 1U;
  }

  for (; i < src_len; i++)
  {
    if (src[i] == '.')
    {
      if (seen_dot)
      {
        return false;
      }
      seen_dot = true;
      continue;
    }

    if (!isdigit((unsigned char)src[i]))
    {
      return false;
    }

    seen_digit = true;
    if (!seen_dot)
    {
      int_part = (int_part * 10) + (int32_t)(src[i] - '0');
    }
    else if (frac_scale < 2)
    {
      frac_part = (frac_part * 10) + (int32_t)(src[i] - '0');
      frac_scale++;
    }
  }

  if (!seen_digit)
  {
    return false;
  }

  while (frac_scale < 2)
  {
    frac_part *= 10;
    frac_scale++;
  }

  *out = sign * ((int_part * 100) + frac_part);
  return true;
}

static bool parse_degmin_to_deg_e7(const char *src, size_t src_len, bool is_longitude, char dir, int32_t *out)
{
  uint32_t whole = 0U;
  uint32_t frac = 0U;
  uint32_t frac_scale = 0U;
  size_t i = 0U;
  uint32_t deg = 0U;
  uint32_t minute_int = 0U;
  uint32_t minute_scaled_1e6 = 0U;
  uint32_t minute_deg_e7 = 0U;
  uint32_t dec_deg_e7 = 0U;
  bool seen_dot = false;
  bool seen_digit = false;
  int sign = 1;

  if (src == NULL || out == NULL || src_len == 0U)
  {
    return false;
  }

  for (i = 0U; i < src_len; i++)
  {
    if (src[i] == '.')
    {
      if (seen_dot)
      {
        return false;
      }
      seen_dot = true;
      continue;
    }

    if (!isdigit((unsigned char)src[i]))
    {
      return false;
    }

    seen_digit = true;
    if (!seen_dot)
    {
      whole = (whole * 10U) + (uint32_t)(src[i] - '0');
    }
    else if (frac_scale < 6U)
    {
      frac = (frac * 10U) + (uint32_t)(src[i] - '0');
      frac_scale++;
    }
  }

  if (!seen_digit)
  {
    return false;
  }

  while (frac_scale < 6U)
  {
    frac *= 10U;
    frac_scale++;
  }

  deg = whole / 100U;
  minute_int = whole % 100U;
  minute_scaled_1e6 = (minute_int * 1000000U) + frac;
  if (minute_scaled_1e6 >= (60U * 1000000U))
  {
    return false;
  }

  /* minute_scaled_1e6 / 6 == (minutes/60)*1e7, with rounding. */
  minute_deg_e7 = (minute_scaled_1e6 + 3U) / 6U;
  dec_deg_e7 = (deg * 10000000U) + minute_deg_e7;

  if (dir == 'W' || dir == 'w' || dir == 'S' || dir == 's')
  {
    sign = -1;
  }
  else if (!(dir == 'E' || dir == 'e' || dir == 'N' || dir == 'n'))
  {
    return false;
  }

  if (is_longitude && dec_deg_e7 > 1800000000U)
  {
    return false;
  }
  if (!is_longitude && dec_deg_e7 > 900000000U)
  {
    return false;
  }

  *out = (sign > 0) ? (int32_t)dec_deg_e7 : -(int32_t)dec_deg_e7;
  return true;
}

static bool extract_csv_field(const char *body, size_t body_len, size_t field_index, const char **out_start, size_t *out_len)
{
  size_t idx = 0U;
  size_t field = 0U;
  size_t start = 0U;

  if (body == NULL || out_start == NULL || out_len == NULL)
  {
    return false;
  }

  for (idx = 0U; idx <= body_len; idx++)
  {
    if (idx == body_len || body[idx] == ',')
    {
      if (field == field_index)
      {
        *out_start = &body[start];
        *out_len = idx - start;
        return true;
      }

      field++;
      start = idx + 1U;
    }
  }

  return false;
}

static void parse_bdpwi_fields(const char *body, size_t body_len, GliderRdssDecodedFrame *out_frame)
{
  const char *field = NULL;
  size_t field_len = 0U;

  if (body == NULL || out_frame == NULL)
  {
    return;
  }

  if (strcmp(out_frame->sentence, "BDPWI") != 0)
  {
    return;
  }

  out_frame->is_bdpwi = true;
  out_frame->bdpwi.valid = true;

  if (extract_csv_field(body, body_len, 1U, &field, &field_len))
  {
    copy_token(out_frame->bdpwi_utc, sizeof(out_frame->bdpwi_utc), field, field_len);
    copy_token(out_frame->bdpwi.utc, sizeof(out_frame->bdpwi.utc), field, field_len);
  }

  if (extract_csv_field(body, body_len, 2U, &field, &field_len))
  {
    out_frame->bdpwi_field1_valid = parse_uint_field(field, field_len, &out_frame->bdpwi_field1);
    out_frame->bdpwi.mode_code_valid = parse_uint_field(field, field_len, &out_frame->bdpwi.mode_code);
  }

  if (extract_csv_field(body, body_len, 3U, &field, &field_len))
  {
    out_frame->bdpwi_field2_valid = parse_uint_field(field, field_len, &out_frame->bdpwi_field2);
    out_frame->bdpwi.status_code_valid = parse_uint_field(field, field_len, &out_frame->bdpwi.status_code);
  }

  if (extract_csv_field(body, body_len, 4U, &field, &field_len))
  {
    copy_token(out_frame->bdpwi.latitude_raw, sizeof(out_frame->bdpwi.latitude_raw), field, field_len);
    out_frame->bdpwi.latitude_valid = parse_decimal_x100(field, field_len, &out_frame->bdpwi.latitude_x100);
  }

  if (extract_csv_field(body, body_len, 5U, &field, &field_len))
  {
    copy_token(out_frame->bdpwi.longitude_raw, sizeof(out_frame->bdpwi.longitude_raw), field, field_len);
    out_frame->bdpwi.longitude_valid = parse_decimal_x100(field, field_len, &out_frame->bdpwi.longitude_x100);
  }

  if (extract_csv_field(body, body_len, 6U, &field, &field_len))
  {
    copy_token(out_frame->bdpwi.heading_raw, sizeof(out_frame->bdpwi.heading_raw), field, field_len);
    out_frame->bdpwi.heading_valid = parse_decimal_x100(field, field_len, &out_frame->bdpwi.heading_x100);
  }
}

static void parse_bdepi_fields(const char *body, size_t body_len, GliderRdssDecodedFrame *out_frame)
{
  const char *field = NULL;
  size_t field_len = 0U;
  char dir = '\0';

  if (body == NULL || out_frame == NULL)
  {
    return;
  }

  if (strcmp(out_frame->sentence, "BDEPI") != 0)
  {
    return;
  }

  out_frame->is_bdepi = true;
  out_frame->bdepi.valid = true;

  if (extract_csv_field(body, body_len, 1U, &field, &field_len))
  {
    copy_token(out_frame->bdepi.sender_card, sizeof(out_frame->bdepi.sender_card), field, field_len);
  }

  if (extract_csv_field(body, body_len, 2U, &field, &field_len))
  {
    copy_token(out_frame->bdepi.receiver_card, sizeof(out_frame->bdepi.receiver_card), field, field_len);
  }

  if (extract_csv_field(body, body_len, 3U, &field, &field_len))
  {
    out_frame->bdepi.freq_code_valid = parse_uint_field(field, field_len, &out_frame->bdepi.freq_code);
  }

  if (extract_csv_field(body, body_len, 4U, &field, &field_len))
  {
    out_frame->bdepi.emergency_code_valid = parse_uint_field(field, field_len, &out_frame->bdepi.emergency_code);
  }

  if (extract_csv_field(body, body_len, 5U, &field, &field_len))
  {
    out_frame->bdepi.report_mode_valid = parse_uint_field(field, field_len, &out_frame->bdepi.report_mode_code);
  }

  if (extract_csv_field(body, body_len, 6U, &field, &field_len))
  {
    copy_token(out_frame->bdepi.utc, sizeof(out_frame->bdepi.utc), field, field_len);
  }

  if (extract_csv_field(body, body_len, 7U, &field, &field_len))
  {
    copy_token(out_frame->bdepi.longitude_dm, sizeof(out_frame->bdepi.longitude_dm), field, field_len);
  }

  if (extract_csv_field(body, body_len, 8U, &field, &field_len))
  {
    copy_token(out_frame->bdepi.longitude_ew, sizeof(out_frame->bdepi.longitude_ew), field, field_len);
    if (field_len > 0U)
    {
      dir = field[0];
      out_frame->bdepi.longitude_valid = parse_degmin_to_deg_e7(
          out_frame->bdepi.longitude_dm,
          bounded_strnlen(out_frame->bdepi.longitude_dm, sizeof(out_frame->bdepi.longitude_dm)),
          true,
          dir,
          &out_frame->bdepi.longitude_deg_e7);
    }
  }

  if (extract_csv_field(body, body_len, 9U, &field, &field_len))
  {
    copy_token(out_frame->bdepi.latitude_dm, sizeof(out_frame->bdepi.latitude_dm), field, field_len);
  }

  if (extract_csv_field(body, body_len, 10U, &field, &field_len))
  {
    copy_token(out_frame->bdepi.latitude_ns, sizeof(out_frame->bdepi.latitude_ns), field, field_len);
    if (field_len > 0U)
    {
      dir = field[0];
      out_frame->bdepi.latitude_valid = parse_degmin_to_deg_e7(
          out_frame->bdepi.latitude_dm,
          bounded_strnlen(out_frame->bdepi.latitude_dm, sizeof(out_frame->bdepi.latitude_dm)),
          false,
          dir,
          &out_frame->bdepi.latitude_deg_e7);
    }
  }

  if (extract_csv_field(body, body_len, 11U, &field, &field_len))
  {
    copy_token(out_frame->bdepi.altitude_raw, sizeof(out_frame->bdepi.altitude_raw), field, field_len);
    out_frame->bdepi.altitude_valid = parse_decimal_x100(field, field_len, &out_frame->bdepi.altitude_x100);
  }
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
  parse_bdpwi_fields(&line[1], body_len, out_frame);
  parse_bdepi_fields(&line[1], body_len, out_frame);

  out_frame->type = GLIDER_RDSS_FRAME_NMEA_TEXT;
  return true;
}

static bool parse_plain_text(const char *line, size_t line_len, GliderRdssDecodedFrame *out_frame)
{
  if (line == NULL || out_frame == NULL || line_len == 0U)
  {
    return false;
  }

  copy_token(out_frame->text, sizeof(out_frame->text), line, line_len);

  if (strcmp(line, "RD Mode!") == 0)
  {
    out_frame->is_rd_mode = true;
  }

  out_frame->type = GLIDER_RDSS_FRAME_PLAIN_TEXT;
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

  if (parse_plain_text(normalized_line, normalized_len, out_frame))
  {
    return true;
  }

  out_frame->type = GLIDER_RDSS_FRAME_UNKNOWN;
  copy_token(out_frame->text, sizeof(out_frame->text), normalized_line, normalized_len);
  return true;
}
