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
  bool valid;                 /* True when sentence is BDPWI and parse path reached */
  char utc[16];               /* Field[1], e.g. "055523.77" */
  uint32_t mode_code;         /* Field[2], mode code */
  uint32_t status_code;       /* Field[3], status code */
  bool mode_code_valid;       /* Field[2] is numeric */
  bool status_code_valid;     /* Field[3] is numeric */
  char latitude_raw[16];      /* Field[4], original latitude text */
  char longitude_raw[16];     /* Field[5], original longitude text */
  char heading_raw[16];       /* Field[6], original heading text */
  int32_t latitude_x100;      /* Field[4] fixed-point value (x100) */
  int32_t longitude_x100;     /* Field[5] fixed-point value (x100) */
  int32_t heading_x100;       /* Field[6] fixed-point value (x100) */
  bool latitude_valid;        /* Field[4] parsed as decimal */
  bool longitude_valid;       /* Field[5] parsed as decimal */
  bool heading_valid;         /* Field[6] parsed as decimal */
} GliderRdssBdpwiInfo;

typedef struct
{
  bool valid;                 /* True when sentence is BDEPI and parse path reached */
  char sender_card[24];       /* Field[1], sender card number */
  char receiver_card[24];     /* Field[2], receiver card number */
  uint32_t freq_code;         /* Field[3], frequency code */
  uint32_t emergency_code;    /* Field[4], emergency code */
  uint32_t report_mode_code;  /* Field[5], report mode code */
  bool freq_code_valid;       /* Field[3] is numeric */
  bool emergency_code_valid;  /* Field[4] is numeric */
  bool report_mode_valid;     /* Field[5] is numeric */
  char utc[16];               /* Field[6], UTC time string */
  char longitude_dm[20];      /* Field[7], dddmm.mmmmmm text */
  char longitude_ew[4];       /* Field[8], E/W */
  char latitude_dm[20];       /* Field[9], ddmm.mmmmmm text */
  char latitude_ns[4];        /* Field[10], N/S */
  int32_t longitude_deg_e7;   /* Converted longitude in degree * 1e7 */
  int32_t latitude_deg_e7;    /* Converted latitude in degree * 1e7 */
  bool longitude_valid;       /* Field[7]+Field[8] converted successfully */
  bool latitude_valid;        /* Field[9]+Field[10] converted successfully */
  char altitude_raw[16];      /* Field[11], altitude text */
  int32_t altitude_x100;      /* Field[11] fixed-point value (x100 meters) */
  bool altitude_valid;        /* Field[11] parsed as decimal */
} GliderRdssBdepiInfo;

typedef struct
{
  GliderRdssFrameType type;   /* High-level classification */
  bool is_rd_mode;            /* True when text exactly equals "RD Mode!" */
  bool is_bdpwi;              /* True when sentence id is BDPWI */
  bool is_bdepi;              /* True when sentence id is BDEPI */
  bool has_checksum;          /* NMEA '*' checksum existed */
  bool checksum_valid;        /* XOR checksum result */
  uint8_t checksum_rx;        /* Checksum from sentence */
  uint8_t checksum_calc;      /* Locally calculated checksum */
  char sentence[24];          /* Sentence id, e.g. "BDPWI" */
  char command[24];           /* AT command token when AT response */
  char bdpwi_utc[16];         /* Backward-compatible UTC mirror */
  uint32_t bdpwi_field1;      /* Backward-compatible Field[2] mirror */
  uint32_t bdpwi_field2;      /* Backward-compatible Field[3] mirror */
  bool bdpwi_field1_valid;    /* Backward-compatible Field[2] validity */
  bool bdpwi_field2_valid;    /* Backward-compatible Field[3] validity */
  GliderRdssBdpwiInfo bdpwi;  /* Detailed BDPWI decode */
  GliderRdssBdepiInfo bdepi;  /* Detailed BDEPI decode */
  char text[192];             /* Full normalized text line */
  uint8_t payload[128];       /* Raw payload bytes/text */
  size_t payload_len;         /* Payload length */
} GliderRdssDecodedFrame;

bool GliderRdssCodec_TryDecodeLine(const char *line, GliderRdssDecodedFrame *out_frame);

#endif
