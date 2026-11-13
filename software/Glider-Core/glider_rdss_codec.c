#include "glider_rdss_codec.h"
#include <math.h>


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

static bool parse_decimal_degrees_e7(
    const char *src,
    size_t src_len,
    uint32_t max_degrees,
    char direction,
    int32_t *out)
{
  size_t i = 0U;
  uint32_t whole = 0U;
  uint32_t fraction = 0U;
  uint32_t fraction_digits = 0U;
  bool seen_dot = false;
  bool seen_digit = false;
  int32_t value = 0;

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
      if (whole > max_degrees)
      {
        return false;
      }
    }
    else
    {
      if (fraction_digits >= 7U)
      {
        return false;
      }
      fraction = (fraction * 10U) + (uint32_t)(src[i] - '0');
      fraction_digits++;
    }
  }

  if (!seen_digit || (seen_dot && fraction_digits == 0U))
  {
    return false;
  }

  while (fraction_digits < 7U)
  {
    fraction *= 10U;
    fraction_digits++;
  }

  if (whole == max_degrees && fraction != 0U)
  {
    return false;
  }

  value = (int32_t)((whole * 10000000U) + fraction);
  if (direction == 'S' || direction == 's' || direction == 'W' || direction == 'w')
  {
    value = -value;
  }

  *out = value;
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
  char dir1 = '\0';
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

static bool parse_bds_command_frame(const char *line, size_t line_len, GliderRdssDecodedFrame *out_frame)
{
  static const char prefix[] = "$BDS:";
  static const char command[] = "go to";
  size_t cursor = 0U;
  size_t card_start = 0U;
  size_t card_len = 0U;
  size_t value_start = 0U;
  size_t value_len = 0U;
  size_t i = 0U;
  char latitude_dir = '\0';
  char longitude_dir = '\0';

  if (line == NULL || out_frame == NULL || line_len == 0U)
  {
    return false;
  }

  if (str_starts_with(line, prefix))
  {
    cursor = sizeof(prefix) - 1U;
    card_start = cursor;
    while (cursor < line_len && line[cursor] != ',')
    {
      if (!is_digit_ascii(line[cursor]))
      {
        return false;
      }
      cursor++;
    }
    card_len = cursor - card_start;
    if (card_len == 0U || card_len >= sizeof(out_frame->bds_command.sender_card) || cursor >= line_len)
    {
      return false;
    }
    cursor++;
  }

  while (cursor < line_len && is_space_ascii(line[cursor]))
  {
    cursor++;
  }
  if ((line_len - cursor) < (sizeof(command) - 1U))
  {
    return false;
  }
  for (i = 0U; i < (sizeof(command) - 1U); i++)
  {
    if (to_lower_ascii(line[cursor + i]) != command[i])
    {
      return false;
    }
  }
  cursor += sizeof(command) - 1U;
  if (cursor >= line_len || !is_space_ascii(line[cursor]))
  {
    return false;
  }
  while (cursor < line_len && is_space_ascii(line[cursor]))
  {
    cursor++;
  }

  if (cursor >= line_len || !(line[cursor] == 'N' || line[cursor] == 'n' ||
                              line[cursor] == 'S' || line[cursor] == 's'))
  {
    return false;
  }
  latitude_dir = line[cursor++];
  value_start = cursor;
  while (cursor < line_len && !is_space_ascii(line[cursor]))
  {
    cursor++;
  }
  value_len = cursor - value_start;
  if (value_len == 0U || value_len >= sizeof(out_frame->bds_command.latitude_raw) ||
      !parse_decimal_degrees_e7(
          &line[value_start], value_len, 90U, latitude_dir,
          &out_frame->bds_command.latitude_deg_e7))
  {
    return false;
  }
  copy_token(out_frame->bds_command.latitude_raw,
             sizeof(out_frame->bds_command.latitude_raw), &line[value_start], value_len);

  while (cursor < line_len && is_space_ascii(line[cursor]))
  {
    cursor++;
  }
  if (cursor >= line_len || !(line[cursor] == 'E' || line[cursor] == 'e' ||
                              line[cursor] == 'W' || line[cursor] == 'w'))
  {
    return false;
  }
  longitude_dir = line[cursor++];
  value_start = cursor;
  while (cursor < line_len && !is_space_ascii(line[cursor]))
  {
    cursor++;
  }
  value_len = cursor - value_start;
  if (value_len == 0U || value_len >= sizeof(out_frame->bds_command.longitude_raw) ||
      !parse_decimal_degrees_e7(
          &line[value_start], value_len, 180U, longitude_dir,
          &out_frame->bds_command.longitude_deg_e7))
  {
    return false;
  }
  copy_token(out_frame->bds_command.longitude_raw,
             sizeof(out_frame->bds_command.longitude_raw), &line[value_start], value_len);

  while (cursor < line_len && is_space_ascii(line[cursor]))
  {
    cursor++;
  }
  if (cursor != line_len)
  {
    return false;
  }

  if (card_len > 0U)
  {
    copy_token(out_frame->bds_command.sender_card,
               sizeof(out_frame->bds_command.sender_card), &line[card_start], card_len);
  }
  out_frame->bds_command.latitude_ns[0] = latitude_dir;
  out_frame->bds_command.latitude_ns[1] = '\0';
  out_frame->bds_command.longitude_ew[0] = longitude_dir;
  out_frame->bds_command.longitude_ew[1] = '\0';
  out_frame->bds_command.valid = true;
  out_frame->is_bds_command = true;
  out_frame->type = GLIDER_RDSS_FRAME_BDS_COMMAND;
  copy_token(out_frame->sentence, sizeof(out_frame->sentence), "BDS", 3U);
  copy_token(out_frame->text, sizeof(out_frame->text), line, line_len);
  copy_token((char *)out_frame->payload, sizeof(out_frame->payload), line, line_len);
  out_frame->payload_len = (line_len < sizeof(out_frame->payload)) ? line_len : sizeof(out_frame->payload) - 1U;
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

  if (parse_bds_command_frame(normalized_line, normalized_len, out_frame))
  {
    return true;
  }

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


/* 以下检查只服务 GNSS 融合，不改变用于诊断和卫星通信的宽松解码。 */
static bool parse_gnss_digits(const char *field, size_t len, uint32_t *value)
{
  size_t i;
  uint32_t result = 0U;
  if (field == NULL || value == NULL || len == 0U || len > 9U) { return false; }
  for (i = 0U; i < len; ++i)
  {
    if (!is_digit_ascii(field[i])) { return false; }
    result = result * 10U + (uint32_t)(field[i] - '0');
  }
  *value = result;
  return true;
}

static bool parse_gnss_utc(const char *field, size_t len, uint32_t *ms_of_day)
{
  uint32_t h, m, sec, fraction = 0U;
  size_t digits;
  /* 本接口的历元分辨率为 1ms；拒绝更细精度，避免静默合并不同历元。
   * 秒为 60 的闰秒暂不融合，不能映射成下一天的零秒。 */
  if (!(len == 6U || (len >= 8U && len <= 10U))) { return false; }
  if (!parse_gnss_digits(field, 2U, &h) ||
      !parse_gnss_digits(field + 2U, 2U, &m) ||
      !parse_gnss_digits(field + 4U, 2U, &sec) ||
      h > 23U || m > 59U || sec > 59U) { return false; }
  if (len > 6U)
  {
    digits = len - 7U;
    if (field[6] != '.' || !parse_gnss_digits(field + 7U, digits, &fraction))
    { return false; }
    while (digits++ < 3U) { fraction *= 10U; }
  }
  *ms_of_day = ((h * 60U + m) * 60U + sec) * 1000U + fraction;
  return true;
}

static bool parse_gnss_date(const char *field, size_t len, uint32_t *date)
{
  static const uint8_t month_days[12] = {31U, 28U, 31U, 30U, 31U, 30U,
                                       31U, 31U, 30U, 31U, 30U, 31U};
  uint32_t day, month, year, limit;
  if (len != 6U || !parse_gnss_digits(field, 2U, &day) ||
      !parse_gnss_digits(field + 2U, 2U, &month) ||
      !parse_gnss_digits(field + 4U, 2U, &year) || month < 1U || month > 12U)
  { return false; }
  year += 2000U;
  limit = month_days[month - 1U];
  if (month == 2U && year % 4U == 0U) { ++limit; }
  if (day < 1U || day > limit) { return false; }
  *date = year * 10000U + month * 100U + day;
  return true;
}

static bool parse_gnss_coordinate(const char *field, size_t len,
                                  const char *direction, size_t dir_len,
                                  bool longitude, int32_t *deg_e7)
{
  const size_t whole_digits = longitude ? 5U : 4U;
  const uint32_t limit = longitude ? 180U : 90U;
  uint32_t whole, fraction = 0U, deg, minute;
  uint64_t scale = 1U, minutes_scaled, result;
  size_t i;
  bool negative;
  if (dir_len != 1U || len < whole_digits) { return false; }
  if (longitude)
  {
    if (direction[0] != 'E' && direction[0] != 'W') { return false; }
    negative = direction[0] == 'W';
  }
  else
  {
    if (direction[0] != 'N' && direction[0] != 'S') { return false; }
    negative = direction[0] == 'S';
  }
  if (!parse_gnss_digits(field, whole_digits, &whole)) { return false; }
  if (len > whole_digits)
  {
    const size_t digits = len - whole_digits - 1U;
    if (field[whole_digits] != '.' ||
        !parse_gnss_digits(field + whole_digits + 1U, digits, &fraction))
    { return false; }
    for (i = 0U; i < digits; ++i) { scale *= 10U; }
  }
  deg = whole / 100U;
  minute = whole % 100U;
  if (minute >= 60U || deg > limit ||
      (deg == limit && (minute != 0U || fraction != 0U))) { return false; }
  minutes_scaled = (uint64_t)minute * scale + fraction;
  result = (uint64_t)deg * 10000000ULL +
           (minutes_scaled * 10000000ULL + 30ULL * scale) / (60ULL * scale);
  *deg_e7 = negative ? -(int32_t)result : (int32_t)result;
  return true;
}

static bool parse_gnss_unsigned_decimal(const char *field, size_t len, float *value)
{
  size_t whole_len = 0U, fraction_len, i;
  uint32_t whole, fraction = 0U, scale = 1U;
  if (field == NULL || value == NULL || len == 0U) { return false; }
  while (whole_len < len && field[whole_len] != '.') { ++whole_len; }
  if (!parse_gnss_digits(field, whole_len, &whole)) { return false; }
  if (whole_len < len)
  {
    fraction_len = len - whole_len - 1U;
    if (!parse_gnss_digits(field + whole_len + 1U, fraction_len, &fraction))
    { return false; }
    for (i = 0U; i < fraction_len; ++i) { scale *= 10U; }
  }
  *value = (float)whole + (float)fraction / (float)scale;
  return isfinite(*value);
}

static void parse_gnss_velocity(const char *body, size_t body_len, GnssSample *sample)
{
  const char *field;
  size_t len;
  float speed_knots, course_deg, speed_mps, course_rad;
  /* 速度是可选观测：空值、坏格式、低速 COG 均不影响独立的位置观测。
   * 0.3m/s 以下方向不可靠，不能用任意 COG 伪造二维速度或艇体航向。 */
  if (!extract_csv_field(body, body_len, 7U, &field, &len) ||
      !parse_gnss_unsigned_decimal(field, len, &speed_knots) ||
      !extract_csv_field(body, body_len, 8U, &field, &len) ||
      !parse_gnss_unsigned_decimal(field, len, &course_deg) || course_deg >= 360.0f)
  { return; }
  speed_mps = speed_knots * (1852.0f / 3600.0f);
  if (!isfinite(speed_mps) || speed_mps < 0.3f) { return; }
  course_rad = course_deg * 0.01745329251994329577f;
  sample->vel_n_mps = speed_mps * cosf(course_rad);
  sample->vel_e_mps = speed_mps * sinf(course_rad);
  sample->velocity_source = GNSS_VELOCITY_RMC;
  sample->validity_flags |= GNSS_VALID_VELOCITY;
}

bool GliderRdssCodec_TryMakeGnssSample(const GliderRdssDecodedFrame *frame,
                                     uint64_t t_rx_us, uint32_t seq,
                                     GnssSample *out_sample)
{
  const char *body, *star, *field, *coordinate, *direction;
  size_t body_len, len, coordinate_len, direction_len;
  bool no_fix;
  GnssSample sample;
  if (out_sample == NULL) { return false; }
  bytes_zero((uint8_t *)out_sample, sizeof(*out_sample));
  bytes_zero((uint8_t *)&sample, sizeof(sample));
  if (frame == NULL || frame->type != GLIDER_RDSS_FRAME_NMEA_TEXT ||
      !frame->is_gnrmc || !frame->has_checksum || !frame->checksum_valid)
  { return false; }
  star = find_char(frame->text, '*');
  if (star == NULL || frame->text[0] != '$') { return false; }
  body = frame->text + 1U;
  body_len = (size_t)(star - body);
  if (!extract_csv_field(body, body_len, 1U, &field, &len) ||
      !parse_gnss_utc(field, len, &sample.utc_ms_of_day) ||
      !extract_csv_field(body, body_len, 9U, &field, &len) ||
      !parse_gnss_date(field, len, &sample.utc_date_yyyymmdd)) { return false; }
  if (!extract_csv_field(body, body_len, 2U, &field, &len) ||
      len != 1U || !(field[0] == 'A' || field[0] == 'V')) { return false; }
  no_fix = field[0] == 'V';
  /* 兼容没有 mode 的旧 RMC。N 明确表示无定位；E/M/S 不作为实测位置。
   * V 事件可携带 E/M/S 模式，但只通知无定位，不发布其中的坐标。 */
  if (extract_csv_field(body, body_len, 12U, &field, &len) && len != 0U)
  {
    if (len != 1U) { return false; }
    if (field[0] == 'N') { no_fix = true; }
    else if (!(field[0] == 'A' || field[0] == 'D' || field[0] == 'F' ||
               field[0] == 'R' || field[0] == 'P' ||
               (no_fix && (field[0] == 'E' || field[0] == 'M' || field[0] == 'S'))))
    { return false; }
  }
  if (extract_csv_field(body, body_len, 13U, &field, &len) && len != 0U)
  {
    if (len != 1U) { return false; }
    if (field[0] == 'V') { no_fix = true; }
    else if (!(field[0] == 'S' || (no_fix && (field[0] == 'C' || field[0] == 'U'))))
    { return false; }
  }
  if (extract_csv_field(body, body_len, 14U, &field, &len)) { return false; }
  sample.seq = seq;
  sample.t_rx_us = t_rx_us;
  if (no_fix)
  {
    sample.validity_flags = GNSS_VALID_CHECKSUM | GNSS_VALID_UTC | GNSS_VALID_NO_FIX;
    *out_sample = sample;
    return true;
  }
  if (!extract_csv_field(body, body_len, 3U, &coordinate, &coordinate_len) ||
      !extract_csv_field(body, body_len, 4U, &direction, &direction_len) ||
      !parse_gnss_coordinate(coordinate, coordinate_len, direction, direction_len,
                             false, &sample.lat_deg_e7) ||
      !extract_csv_field(body, body_len, 5U, &coordinate, &coordinate_len) ||
      !extract_csv_field(body, body_len, 6U, &direction, &direction_len) ||
      !parse_gnss_coordinate(coordinate, coordinate_len, direction, direction_len,
                             true, &sample.lon_deg_e7)) { return false; }
  sample.validity_flags = GNSS_VALID_REQUIRED;
  parse_gnss_velocity(body, body_len, &sample);
  *out_sample = sample;
  return true;
}
