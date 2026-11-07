#ifndef FRAMING_H
#define FRAMING_H

#include <stddef.h>
#include <stdint.h>
#include "cobs-c/cobs.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FRM_BYTE_DELIMITER      0x00

#define FRM_BYTE_SIGNATURE_1    0x31 /* M31 family */
#define FRM_BYTE_SIGNATURE_2    0xEE /* Enterprise */

#define FRM_SIGNATURE_LEN       2
#define FRM_CRC_LEN             2
#define FRM_OVERHEAD            (FRM_SIGNATURE_LEN + FRM_CRC_LEN)

#define FRM_IDX_SIGNATURE       0
#define FRM_IDX_PAYLOAD         (FRM_IDX_SIGNATURE + FRM_SIGNATURE_LEN)

#ifndef FRM_MAX_PAYLOAD_LEN
#error "FRM_MAX_PAYLOAD_LEN must be defined"
#endif

#if FRM_MAX_PAYLOAD_LEN > 65533
#error "FRM_MAX_PAYLOAD_LEN must be <= 65533"
#endif

#define FRM_MAX_RAW_LEN         (FRM_OVERHEAD + FRM_MAX_PAYLOAD_LEN)
#define FRM_MAX_ENCODED_LEN     COBS_ENCODE_DST_BUF_LEN_MAX(FRM_MAX_RAW_LEN)
#define FRM_MAX_FRAME_LEN       (FRM_MAX_ENCODED_LEN + 1)

typedef enum {
        FRM_CRC_ENABLED = 0,
        FRM_CRC_DISABLED
} frm_crc_mode_t;

typedef struct {
        uint8_t  buffer[FRM_MAX_FRAME_LEN];
        uint32_t frame_len;
        uint16_t payload_len;
} frm_frame_t;

typedef enum {
        FRM_BUILD_SUCCESS = 0,
        FRM_BUILD_ERROR_NULL_ARG,
        FRM_BUILD_ERROR_PAYLOAD_TOO_LONG,
        FRM_BUILD_ERROR_ENCODE
} frm_build_return_t;

typedef enum {
        FRM_PARSER_STATE_RESET = 0,
        FRM_PARSER_STATE_RECEIVING,
        FRM_PARSER_STATE_DISCARD
} frm_parser_state_t;

typedef struct {
        frm_parser_state_t state;
        uint32_t           encoded_len;
        uint8_t            encoded[FRM_MAX_ENCODED_LEN];
} frm_parser_ctx_t;

typedef struct {
        frm_crc_mode_t   crc_mode;
        frm_parser_ctx_t parser;
        uint8_t          rx_raw[FRM_MAX_RAW_LEN];

        struct {
                uint32_t parser_frames_ok;
                uint32_t parser_err_crc;
                uint32_t parser_err_len;
                uint32_t parser_err_signature;
                uint32_t parser_err_cobs;
                uint32_t parser_err_nullarg;
        } debug;
} frm_ctx_t;

/* ================================================================================================
 * API area
 * ================================================================================================ */

void frm_ctx_init(frm_ctx_t *ctx, frm_crc_mode_t crc_mode);

void frm_ctx_reset(frm_ctx_t *ctx);

frm_build_return_t frm_build(
        const frm_ctx_t *ctx,
        const uint8_t    payload[],
        const size_t     payload_len,
        frm_frame_t     *out_frame
);

typedef void (*frm_on_frame_t)(const uint8_t *payload, uint16_t payload_len, void *user);

uint32_t frm_feed(
        frm_ctx_t      *ctx,
        const uint8_t  *in_buf,
        const size_t    in_buf_len,
        frm_on_frame_t  on_frame,
        void           *user
);

#ifdef __cplusplus
}
#endif

#endif /* FRAMING_H */
