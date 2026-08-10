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
  GLIDER_RDSS_FRAME_HEX_BINARY,
  GLIDER_RDSS_FRAME_BDS_COMMAND
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
  char latitude_dm[20];       /* Located latitude candidate in ddmm.mmmmmm form */
  char latitude_ns[4];        /* Located latitude direction candidate */
  char longitude_dm[20];      /* Located longitude candidate in dddmm.mmmmmm form */
  char longitude_ew[4];       /* Located longitude direction candidate */
  int32_t latitude_deg_e7;    /* Converted latitude in degree * 1e7 */
  int32_t longitude_deg_e7;   /* Converted longitude in degree * 1e7 */
  bool latitude_deg_valid;    /* Located latitude candidate converted successfully */
  bool longitude_deg_valid;   /* Located longitude candidate converted successfully */
  uint32_t old_beam_count;    /* Old-system locked beam count */
  uint32_t new_beam_count;    /* New-system locked beam count */
  uint32_t best_d_cnr;        /* Best D-branch CNR in this sentence */
  uint32_t best_p_cnr;        /* Best P-branch CNR in this sentence */
  uint32_t good_beam_count;   /* Count of beams with good D/P CNR */
  bool signal_good;           /* True when beam quality passes threshold */
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
  char altitude_unit[4];      /* Field[12], altitude unit */
  char altitude_abnormal[8];  /* Field[13], altitude abnormal flag/value */
  char altitude_abnormal_unit[4]; /* Field[14], altitude abnormal unit */
  char precision_indicator[8]; /* Field[15], precision indicator */
  char multi_solution[8];     /* Field[16], multi-solution indicator */
  char rescue_subtype[8];     /* Field[17], rescue service subtype */
  char rescue_center_type[8]; /* Field[18], rescue center indication type */
  uint32_t data_len;          /* Field[19], rescue/status data length */
  bool data_len_valid;        /* Field[19] is numeric */
  char data_payload[48];      /* Field[20], rescue/status data payload */
} GliderRdssBdepiInfo;

typedef struct
{
  bool valid;                 /* True when sentence is GNRMC and parse path reached */
  char utc[16];               /* Field[1], UTC time string */
  char status[4];             /* Field[2], A/V */
  char latitude_dm[20];       /* Field[3], ddmm.mmmm text */
  char latitude_ns[4];        /* Field[4], N/S */
  char longitude_dm[20];      /* Field[5], dddmm.mmmm text */
  char longitude_ew[4];       /* Field[6], E/W */
  int32_t latitude_deg_e7;    /* Converted latitude in degree * 1e7 */
  int32_t longitude_deg_e7;   /* Converted longitude in degree * 1e7 */
  bool latitude_valid;        /* Latitude converted successfully */
  bool longitude_valid;       /* Longitude converted successfully */
  char date[16];              /* Field[9], ddmmyy */
  bool status_valid;          /* True when status is A */
} GliderRdssGnrmcInfo;

typedef struct
{
  bool valid;                 /* True for a complete "go to" command */
  char sender_card[24];       /* Card number after "$BDS:", empty for payload-only RX */
  char latitude_raw[20];      /* Decimal-degree latitude text */
  char latitude_ns[4];        /* N/S */
  char longitude_raw[20];     /* Decimal-degree longitude text */
  char longitude_ew[4];       /* E/W */
  int32_t latitude_deg_e7;    /* Signed latitude in degree * 1e7 */
  int32_t longitude_deg_e7;   /* Signed longitude in degree * 1e7 */
} GliderRdssBdsCommandInfo;

typedef struct
{
  GliderRdssFrameType type;   /* High-level classification */
  bool is_rd_mode;            /* True when text exactly equals "RD Mode!" */
  bool is_rn_mode;            /* True when text exactly equals RNMODE!/RN Mode! */
  bool is_bdpwi;              /* True when sentence id is BDPWI */
  bool is_bdepi;              /* True when sentence id is BDEPI */
  bool is_gnrmc;              /* True when sentence id is GNRMC */
  bool is_bds_command;        /* True for an enveloped or payload-only go-to command */
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
  GliderRdssGnrmcInfo gnrmc;  /* Detailed GNRMC decode */
  GliderRdssBdsCommandInfo bds_command; /* Ground-station target command */
  char text[512];             /* Full normalized text line */
  uint8_t payload[128];       /* Raw payload bytes/text */
  size_t payload_len;         /* Payload length */
} GliderRdssDecodedFrame;

bool GliderRdssCodec_TryDecodeLine(const char *line, GliderRdssDecodedFrame *out_frame);

#endif
