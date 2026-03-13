#include "glider_rdss_codec.h"


static bool is_digit_ascii(char c)
{
  return (c >= '0' && c <= '9');
}

static bool is_hex_alpha_ascii(char c)
{
  return ((c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F'));
}

static bool is_hex_digit_ascii(char c)
{
  return is_digit_ascii(c) || is_hex_alpha_ascii(c);
}

static char to_lower_ascii(char c)
{
  if (c >= 'A' && c <= 'Z')
  {
    return (char)(c - 'A' + 'a');
  }
  return c;
}

static bool is_space_ascii(char c)
{
  return (c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\v' || c == '\f');
}

static void bytes_copy(uint8_t *dst, const uint8_t *src, size_t len)
{
  size_t i = 0U;

  if (dst == NULL || src == NULL)
  {
    return;
  }

  for (i = 0U; i < len; i++)
  {
    dst[i] = src[i];
  }
}

static void bytes_zero(uint8_t *dst, size_t len)
{
  size_t i = 0U;

  if (dst == NULL)
  {
    return;
  }

  for (i = 0U; i < len; i++)
  {
    dst[i] = 0U;
  }
}

static bool str_equal(const char *a, const char *b)
{
  size_t i = 0U;

  if (a == NULL || b == NULL)
  {
    return false;
  }

  while (a[i] != '\0' && b[i] != '\0')
  {
    if (a[i] != b[i])
    {
      return false;
    }
    i++;
  }

  return (a[i] == '\0' && b[i] == '\0');
}

static bool str_starts_with(const char *s, const char *prefix)
{
  size_t i = 0U;

  if (s == NULL || prefix == NULL)
  {
    return false;
  }

  while (prefix[i] != '\0')
  {
    if (s[i] != prefix[i])
    {
      return false;
    }
    i++;
  }

  return true;
}

static const char *find_char(const char *s, char c)
{
  size_t i = 0U;

  if (s == NULL)
  {
    return NULL;
  }

  while (s[i] != '\0')
  {
    if (s[i] == c)
    {
      return &s[i];
    }
    i++;
  }

  return NULL;
}

static const char *find_char_n(const char *s, size_t len, char c)
{
  size_t i = 0U;

  if (s == NULL)
  {
    return NULL;
  }

  for (i = 0U; i < len; i++)
  {
    if (s[i] == c)
    {
      return &s[i];
    }
  }

  return NULL;
}

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
    bytes_copy((uint8_t *)dst, (const uint8_t *)src, copy_len);
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

  if (!is_hex_digit_ascii(hi) || !is_hex_digit_ascii(lo))
  {
    return false;
  }

  value_hi = is_digit_ascii(hi) ? (hi - '0') : (to_lower_ascii(hi) - 'a' + 10);
  value_lo = is_digit_ascii(lo) ? (lo - '0') : (to_lower_ascii(lo) - 'a' + 10);

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
    if (!is_digit_ascii(src[i]))
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

    if (!is_digit_ascii(src[i]))
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

    if (!is_digit_ascii(src[i]))
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

static bool is_single_dir_field(const char *src, size_t src_len, char positive_dir, char negative_dir, char *out_dir)
{
  char ch = '\0';

  if (src == NULL || out_dir == NULL || src_len != 1U)
  {
    return false;
  }

  ch = src[0];
  if (ch == positive_dir || ch == negative_dir || ch == to_lower_ascii(positive_dir) || ch == to_lower_ascii(negative_dir))
  {
    *out_dir = ch;
    return true;
  }

  return false;
}

static bool looks_like_degmin_field(const char *src, size_t src_len, bool is_longitude)
{
  size_t i = 0U;
  size_t dot_index = src_len;
  size_t whole_digits = 0U;
  uint32_t whole = 0U;
  uint32_t minute_int = 0U;

  if (src == NULL || src_len == 0U)
  {
    return false;
  }

  for (i = 0U; i < src_len; i++)
  {
    if (src[i] == '.')
    {
      if (dot_index != src_len)
      {
        return false;
      }
      dot_index = i;
      continue;
    }

    if (!is_digit_ascii(src[i]))
    {
      return false;
    }

    if (dot_index == src_len)
    {
      whole = (whole * 10U) + (uint32_t)(src[i] - '0');
      whole_digits++;
    }
  }

  if (whole_digits < (is_longitude ? 5U : 4U))
  {
    return false;
  }

  minute_int = whole % 100U;
  if (minute_int >= 60U)
  {
    return false;
  }

  if (is_longitude)
  {
    return ((whole / 100U) <= 180U);
  }

  return ((whole / 100U) <= 90U);
}

static bool try_parse_bdpwi_coordinate_pair(
    const char *first_field,
    size_t first_len,
    char first_dir,
    bool first_is_longitude,
    const char *second_field,
    size_t second_len,
    char second_dir,
    bool second_is_longitude,
    GliderRdssDecodedFrame *out_frame)
{
  int32_t first_deg_e7 = 0;
  int32_t second_deg_e7 = 0;

  if (first_field == NULL || second_field == NULL || out_frame == NULL)
  {
    return false;
  }

  if (!parse_degmin_to_deg_e7(first_field, first_len, first_is_longitude, first_dir, &first_deg_e7))
  {
    return false;
  }

  if (!parse_degmin_to_deg_e7(second_field, second_len, second_is_longitude, second_dir, &second_deg_e7))
  {
    return false;
  }

  if (first_is_longitude)
  {
    copy_token(out_frame->bdpwi.longitude_dm, sizeof(out_frame->bdpwi.longitude_dm), first_field, first_len);
    out_frame->bdpwi.longitude_ew[0] = first_dir;
    out_frame->bdpwi.longitude_ew[1] = '\0';
    out_frame->bdpwi.longitude_deg_e7 = first_deg_e7;
    out_frame->bdpwi.longitude_deg_valid = true;

    copy_token(out_frame->bdpwi.latitude_dm, sizeof(out_frame->bdpwi.latitude_dm), second_field, second_len);
    out_frame->bdpwi.latitude_ns[0] = second_dir;
    out_frame->bdpwi.latitude_ns[1] = '\0';
    out_frame->bdpwi.latitude_deg_e7 = second_deg_e7;
    out_frame->bdpwi.latitude_deg_valid = true;
  }
  else
  {
    copy_token(out_frame->bdpwi.latitude_dm, sizeof(out_frame->bdpwi.latitude_dm), first_field, first_len);
    out_frame->bdpwi.latitude_ns[0] = first_dir;
    out_frame->bdpwi.latitude_ns[1] = '\0';
    out_frame->bdpwi.latitude_deg_e7 = first_deg_e7;
    out_frame->bdpwi.latitude_deg_valid = true;

    copy_token(out_frame->bdpwi.longitude_dm, sizeof(out_frame->bdpwi.longitude_dm), second_field, second_len);
    out_frame->bdpwi.longitude_ew[0] = second_dir;
    out_frame->bdpwi.longitude_ew[1] = '\0';
    out_frame->bdpwi.longitude_deg_e7 = second_deg_e7;
    out_frame->bdpwi.longitude_deg_valid = true;
  }

  return true;
}

static void analyze_bdpwi_signal_fields(const char *body, size_t body_len, GliderRdssDecodedFrame *out_frame)
{
  const uint32_t min_good_cnr = 30U;
  const char *field = NULL;
  size_t field_len = 0U;
  size_t beam_base = 3U;
  size_t beam_index = 0U;
  uint32_t value = 0U;
  uint32_t d_cnr = 0U;
  uint32_t p_cnr = 0U;
  uint32_t good_beam_count = 0U;

  if (body == NULL || out_frame == NULL)
  {
    return;
  }

  if (extract_csv_field(body, body_len, 2U, &field, &field_len) &&
      parse_uint_field(field, field_len, &out_frame->bdpwi.old_beam_count))
  {
    for (beam_index = 0U; beam_index < out_frame->bdpwi.old_beam_count; beam_index++)
    {
      d_cnr = 0U;
      p_cnr = 0U;

      if (extract_csv_field(body, body_len, beam_base + 1U, &field, &field_len) &&
          parse_uint_field(field, field_len, &d_cnr) &&
          d_cnr > out_frame->bdpwi.best_d_cnr)
      {
        out_frame->bdpwi.best_d_cnr = d_cnr;
      }

      if (extract_csv_field(body, body_len, beam_base + 2U, &field, &field_len) &&
          parse_uint_field(field, field_len, &p_cnr) &&
          p_cnr > out_frame->bdpwi.best_p_cnr)
      {
        out_frame->bdpwi.best_p_cnr = p_cnr;
      }

      if (d_cnr >= min_good_cnr && p_cnr >= min_good_cnr)
      {
        good_beam_count++;
      }

      beam_base += 3U;
    }
  }

  if (extract_csv_field(body, body_len, beam_base, &field, &field_len) &&
      parse_uint_field(field, field_len, &value))
  {
    out_frame->bdpwi.new_beam_count = value;
    beam_base++;

    for (beam_index = 0U; beam_index < out_frame->bdpwi.new_beam_count; beam_index++)
    {
      d_cnr = 0U;
      p_cnr = 0U;

      if (extract_csv_field(body, body_len, beam_base + 1U, &field, &field_len) &&
          parse_uint_field(field, field_len, &d_cnr) &&
          d_cnr > out_frame->bdpwi.best_d_cnr)
      {
        out_frame->bdpwi.best_d_cnr = d_cnr;
      }

      if (extract_csv_field(body, body_len, beam_base + 2U, &field, &field_len) &&
          parse_uint_field(field, field_len, &p_cnr) &&
          p_cnr > out_frame->bdpwi.best_p_cnr)
      {
        out_frame->bdpwi.best_p_cnr = p_cnr;
      }

      if (d_cnr >= min_good_cnr && p_cnr >= min_good_cnr)
      {
        good_beam_count++;
      }

      beam_base += 3U;
    }
  }

  out_frame->bdpwi.good_beam_count = good_beam_count;
  out_frame->bdpwi.signal_good = (good_beam_count > 0U);
}

static void locate_bdpwi_coordinates(const char *body, size_t body_len, GliderRdssDecodedFrame *out_frame)
{
  size_t field_index = 4U;
  const char *field0 = NULL;
  const char *field1 = NULL;
  const char *field2 = NULL;
  const char *field3 = NULL;
  size_t len0 = 0U;
  size_t len1 = 0U;
  size_t len2 = 0U;
  size_t len3 = 0U;
  char dir0 = '\0';
  char dir1 = '\0';
  char dir2 = '\0';
  char dir3 = '\0';

  if (body == NULL || out_frame == NULL)
  {
    return;
  }

  for (field_index = 4U; field_index <= 28U; field_index++)
  {
    if (!extract_csv_field(body, body_len, field_index, &field0, &len0))
    {
      break;
    }

    if (!looks_like_degmin_field(field0, len0, false) && !looks_like_degmin_field(field0, len0, true))
    {
      continue;
    }

    if (extract_csv_field(body, body_len, field_index + 1U, &field1, &len1) &&
        extract_csv_field(body, body_len, field_index + 2U, &field2, &len2) &&
        extract_csv_field(body, body_len, field_index + 3U, &field3, &len3))
    {
      if (looks_like_degmin_field(field0, len0, false) &&
          is_single_dir_field(field1, len1, 'N', 'S', &dir1) &&
          looks_like_degmin_field(field2, len2, true) &&
          is_single_dir_field(field3, len3, 'E', 'W', &dir3) &&
          try_parse_bdpwi_coordinate_pair(field0, len0, dir1, false, field2, len2, dir3, true, out_frame))
      {
        return;
      }

      if (looks_like_degmin_field(field0, len0, true) &&
          is_single_dir_field(field1, len1, 'E', 'W', &dir1) &&
          looks_like_degmin_field(field2, len2, false) &&
          is_single_dir_field(field3, len3, 'N', 'S', &dir3) &&
          try_parse_bdpwi_coordinate_pair(field0, len0, dir1, true, field2, len2, dir3, false, out_frame))
      {
        return;
      }
    }

    if (extract_csv_field(body, body_len, field_index + 1U, &field1, &len1))
    {
      if (looks_like_degmin_field(field0, len0, false) &&
          looks_like_degmin_field(field1, len1, true) &&
          try_parse_bdpwi_coordinate_pair(field0, len0, 'N', false, field1, len1, 'E', true, out_frame))
      {
        return;
      }

      if (looks_like_degmin_field(field0, len0, true) &&
          looks_like_degmin_field(field1, len1, false) &&
          try_parse_bdpwi_coordinate_pair(field0, len0, 'E', true, field1, len1, 'N', false, out_frame))
      {
        return;
      }
    }
  }
}

static void parse_bdpwi_fields(const char *body, size_t body_len, GliderRdssDecodedFrame *out_frame)
{
  const char *field = NULL;
  size_t field_len = 0U;

  if (body == NULL || out_frame == NULL)
  {
    return;
  }

  if (!str_equal(out_frame->sentence, "BDPWI"))
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

  analyze_bdpwi_signal_fields(body, body_len, out_frame);
  locate_bdpwi_coordinates(body, body_len, out_frame);
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

  if (!str_equal(out_frame->sentence, "BDEPI"))
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

  if (extract_csv_field(body, body_len, 12U, &field, &field_len))
  {
    copy_token(out_frame->bdepi.altitude_unit, sizeof(out_frame->bdepi.altitude_unit), field, field_len);
  }

  if (extract_csv_field(body, body_len, 13U, &field, &field_len))
  {
    copy_token(out_frame->bdepi.altitude_abnormal, sizeof(out_frame->bdepi.altitude_abnormal), field, field_len);
  }

  if (extract_csv_field(body, body_len, 14U, &field, &field_len))
  {
    copy_token(out_frame->bdepi.altitude_abnormal_unit, sizeof(out_frame->bdepi.altitude_abnormal_unit), field, field_len);
  }

  if (extract_csv_field(body, body_len, 15U, &field, &field_len))
  {
    copy_token(out_frame->bdepi.precision_indicator, sizeof(out_frame->bdepi.precision_indicator), field, field_len);
  }

  if (extract_csv_field(body, body_len, 16U, &field, &field_len))
  {
    copy_token(out_frame->bdepi.multi_solution, sizeof(out_frame->bdepi.multi_solution), field, field_len);
  }

  if (extract_csv_field(body, body_len, 17U, &field, &field_len))
  {
    copy_token(out_frame->bdepi.rescue_subtype, sizeof(out_frame->bdepi.rescue_subtype), field, field_len);
  }

  if (extract_csv_field(body, body_len, 18U, &field, &field_len))
  {
    copy_token(out_frame->bdepi.rescue_center_type, sizeof(out_frame->bdepi.rescue_center_type), field, field_len);
  }

  if (extract_csv_field(body, body_len, 19U, &field, &field_len))
  {
    out_frame->bdepi.data_len_valid = parse_uint_field(field, field_len, &out_frame->bdepi.data_len);
  }

  if (extract_csv_field(body, body_len, 20U, &field, &field_len))
  {
    copy_token(out_frame->bdepi.data_payload, sizeof(out_frame->bdepi.data_payload), field, field_len);
  }
}

static void parse_gnrmc_fields(const char *body, size_t body_len, GliderRdssDecodedFrame *out_frame)
{
  const char *field = NULL;
  size_t field_len = 0U;
  char dir = '\0';

  if (body == NULL || out_frame == NULL)
  {
    return;
  }

  if (!str_equal(out_frame->sentence, "GNRMC"))
  {
    return;
  }

  out_frame->is_gnrmc = true;
  out_frame->gnrmc.valid = true;

  if (extract_csv_field(body, body_len, 1U, &field, &field_len))
  {
    copy_token(out_frame->gnrmc.utc, sizeof(out_frame->gnrmc.utc), field, field_len);
  }

  if (extract_csv_field(body, body_len, 2U, &field, &field_len))
  {
    copy_token(out_frame->gnrmc.status, sizeof(out_frame->gnrmc.status), field, field_len);
    out_frame->gnrmc.status_valid = (field_len > 0U && (field[0] == 'A' || field[0] == 'a'));
  }

  if (extract_csv_field(body, body_len, 3U, &field, &field_len))
  {
    copy_token(out_frame->gnrmc.latitude_dm, sizeof(out_frame->gnrmc.latitude_dm), field, field_len);
  }

  if (extract_csv_field(body, body_len, 4U, &field, &field_len))
  {
    copy_token(out_frame->gnrmc.latitude_ns, sizeof(out_frame->gnrmc.latitude_ns), field, field_len);
    if (field_len > 0U)
    {
      dir = field[0];
      out_frame->gnrmc.latitude_valid = parse_degmin_to_deg_e7(
          out_frame->gnrmc.latitude_dm,
          bounded_strnlen(out_frame->gnrmc.latitude_dm, sizeof(out_frame->gnrmc.latitude_dm)),
          false,
          dir,
          &out_frame->gnrmc.latitude_deg_e7);
    }
  }

  if (extract_csv_field(body, body_len, 5U, &field, &field_len))
  {
    copy_token(out_frame->gnrmc.longitude_dm, sizeof(out_frame->gnrmc.longitude_dm), field, field_len);
  }

  if (extract_csv_field(body, body_len, 6U, &field, &field_len))
  {
    copy_token(out_frame->gnrmc.longitude_ew, sizeof(out_frame->gnrmc.longitude_ew), field, field_len);
    if (field_len > 0U)
    {
      dir = field[0];
      out_frame->gnrmc.longitude_valid = parse_degmin_to_deg_e7(
          out_frame->gnrmc.longitude_dm,
          bounded_strnlen(out_frame->gnrmc.longitude_dm, sizeof(out_frame->gnrmc.longitude_dm)),
          true,
          dir,
          &out_frame->gnrmc.longitude_deg_e7);
    }
  }

  if (extract_csv_field(body, body_len, 9U, &field, &field_len))
  {
    copy_token(out_frame->gnrmc.date, sizeof(out_frame->gnrmc.date), field, field_len);
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

  asterisk = find_char(line, '*');
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

  comma = find_char_n(&line[1], body_len, ',');
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
  parse_gnrmc_fields(&line[1], body_len, out_frame);

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

  if (str_equal(line, "RD Mode!"))
  {
    out_frame->is_rd_mode = true;
  }

  if (str_equal(line, "RNMODE!") || str_equal(line, "RN Mode!"))
  {
    out_frame->is_rn_mode = true;
  }

  out_frame->type = GLIDER_RDSS_FRAME_PLAIN_TEXT;
  return true;
}

static bool parse_hex_line(const char *line, size_t line_len, GliderRdssDecodedFrame *out_frame)
{
  size_t i = 0;
  int pending_nibble = -1;

  if (line_len < 4U || !str_starts_with(line, "HEX:"))
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

    if (!is_hex_digit_ascii((char)ch))
    {
      return false;
    }

    nibble = is_digit_ascii((char)ch) ? (ch - '0') : (to_lower_ascii((char)ch) - 'a' + 10);
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

  if (str_starts_with(line, "AT") || str_starts_with(line, "OK") || str_starts_with(line, "ERROR"))
  {
    out_frame->type = GLIDER_RDSS_FRAME_AT_RESPONSE;
    copy_token(out_frame->command, sizeof(out_frame->command), line, line_len);
    return true;
  }

  plus = find_char(line, '+');
  if (plus != NULL)
  {
    sep = find_char(plus, ':');
    if (sep == NULL)
    {
      sep = find_char(plus, ',');
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
  char normalized_line[512];
  size_t normalized_len = 0;

  if (line == NULL || out_frame == NULL)
  {
    return false;
  }

  bytes_zero((uint8_t *)out_frame, sizeof(*out_frame));
  line_len = bounded_strnlen(line, sizeof(normalized_line) - 1U);

  if (line_len == 0)
  {
    return false;
  }

  start = line;
  end = line + line_len;

  while (start < end && is_space_ascii(*start))
  {
    start++;
  }

  while (end > start && is_space_ascii(*(end - 1)))
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
