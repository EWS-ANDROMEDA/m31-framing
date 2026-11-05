#include <string.h>
#include "framing.h"

/* Internal result of frm_finish_frame(). Errors are reported to users through
 * ctx->debug counters, not return values. */
typedef enum {
        FRM_PARSER_SUCCESS = 0,
        FRM_PARSER_ERROR_INVALID_SIGNATURE,
        FRM_PARSER_ERROR_INVALID_LEN,
        FRM_PARSER_ERROR_COBS_DECODE,
        FRM_PARSER_ERROR_CRC_MISMATCH
} frm_parser_return_t;

/* CRC computation
 * Source: https://docs.robotis.com/docs/dxl/protocol/crc
 */
static unsigned short update_crc(
        unsigned short crc_accum,
        unsigned char *data_blk_ptr,
        unsigned short data_blk_size
){
        unsigned short i, j;
        static const unsigned short crc_table[256] = {
                0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
                0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
                0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
                0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
                0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
                0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
                0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
                0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
                0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
                0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
                0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
                0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
                0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
                0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
                0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
                0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
                0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
                0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
                0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
                0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
                0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
                0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
                0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
                0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
                0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
                0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
                0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
                0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
                0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
                0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
                0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
                0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
        };

        for(j = 0; j < data_blk_size; j++)
        {
                i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
                crc_accum = (crc_accum << 8) ^ crc_table[i];
        }

        return crc_accum;
}

static uint16_t frm_crc16(const uint8_t *data, size_t len)
{
        return (uint16_t)update_crc(0, (unsigned char *)(uintptr_t)data, (unsigned short)len);
}

/* ================================================================================================
 * Context
 * ================================================================================================ */

void frm_ctx_init(frm_ctx_t *ctx, frm_crc_mode_t crc_mode)
{
        if (ctx == NULL) {
                return;
        }
        memset(ctx, 0, sizeof(*ctx));
        ctx->crc_mode = crc_mode;
        ctx->parser.state = FRM_PARSER_STATE_RESET;
}

void frm_ctx_reset(frm_ctx_t *ctx)
{
        if (ctx == NULL) {
                return;
        }
        ctx->parser.state = FRM_PARSER_STATE_RESET;
        ctx->parser.encoded_len = 0;
}

/* ================================================================================================
 * TX
 * ================================================================================================ */

frm_build_return_t frm_build(
        const frm_ctx_t *ctx,
        const uint8_t    payload[],
        const size_t     payload_len,
        frm_frame_t     *out_frame
){
        uint8_t raw[FRM_MAX_RAW_LEN];
        size_t  raw_len;
        cobs_encode_result enc;

        if (ctx == NULL || out_frame == NULL || (payload == NULL && payload_len > 0)) {
                return FRM_BUILD_ERROR_NULL_ARG;
        }
        if (payload_len > FRM_MAX_PAYLOAD_LEN) {
                return FRM_BUILD_ERROR_PAYLOAD_TOO_LONG;
        }

        raw[FRM_IDX_SIGNATURE]     = FRM_BYTE_SIGNATURE_1;
        raw[FRM_IDX_SIGNATURE + 1] = FRM_BYTE_SIGNATURE_2;
        if (payload_len > 0) {
                memcpy(&raw[FRM_IDX_PAYLOAD], payload, payload_len);
        }
        raw_len = FRM_IDX_PAYLOAD + payload_len;

        if (ctx->crc_mode == FRM_CRC_ENABLED) {
                uint16_t crc = frm_crc16(raw, raw_len);
                raw[raw_len++] = (uint8_t)(crc & 0xFF);
                raw[raw_len++] = (uint8_t)(crc >> 8);
        }

        enc = cobs_encode(out_frame->buffer, FRM_MAX_ENCODED_LEN, raw, raw_len);
        if (enc.status != COBS_ENCODE_OK) {
                return FRM_BUILD_ERROR_ENCODE;
        }

        out_frame->buffer[enc.out_len] = FRM_BYTE_DELIMITER;
        out_frame->frame_len   = (uint32_t)(enc.out_len + 1);
        out_frame->payload_len = (uint16_t)payload_len;
        return FRM_BUILD_SUCCESS;
}

/* ================================================================================================
 * RX
 * ================================================================================================ */

/* Decodes and validates parser.encoded. On success the payload is at
 * rx_raw[FRM_IDX_PAYLOAD] and its length is written to *payload_len. */
static frm_parser_return_t frm_finish_frame(frm_ctx_t *ctx, uint16_t *payload_len)
{
        const size_t enc_len = ctx->parser.encoded_len;
        const size_t min_len = FRM_SIGNATURE_LEN + ((ctx->crc_mode == FRM_CRC_ENABLED) ? FRM_CRC_LEN : 0);
        cobs_decode_result dec;
        size_t raw_len;

        ctx->parser.state = FRM_PARSER_STATE_RESET;
        ctx->parser.encoded_len = 0;

        if (enc_len > FRM_MAX_ENCODED_LEN) {
                ctx->debug.parser_err_len++;
                return FRM_PARSER_ERROR_INVALID_LEN;
        }

        dec = cobs_decode(ctx->rx_raw, sizeof(ctx->rx_raw), ctx->parser.encoded, enc_len);
        if (dec.status != COBS_DECODE_OK) {
                ctx->debug.parser_err_cobs++;
                return FRM_PARSER_ERROR_COBS_DECODE;
        }
        raw_len = dec.out_len;

        if (raw_len < min_len) {
                ctx->debug.parser_err_len++;
                return FRM_PARSER_ERROR_INVALID_LEN;
        }

        if (ctx->rx_raw[FRM_IDX_SIGNATURE]     != FRM_BYTE_SIGNATURE_1 ||
            ctx->rx_raw[FRM_IDX_SIGNATURE + 1] != FRM_BYTE_SIGNATURE_2) {
                ctx->debug.parser_err_signature++;
                return FRM_PARSER_ERROR_INVALID_SIGNATURE;
        }

        if (ctx->crc_mode == FRM_CRC_ENABLED) {
                const size_t body_len = raw_len - FRM_CRC_LEN;
                const uint16_t rx_crc = (uint16_t)(ctx->rx_raw[body_len] |
                                                   ((uint16_t)ctx->rx_raw[body_len + 1] << 8));
                if (frm_crc16(ctx->rx_raw, body_len) != rx_crc) {
                        ctx->debug.parser_err_crc++;
                        return FRM_PARSER_ERROR_CRC_MISMATCH;
                }
                raw_len = body_len;
        }

        *payload_len = (uint16_t)(raw_len - FRM_SIGNATURE_LEN);
        ctx->debug.parser_frames_ok++;
        return FRM_PARSER_SUCCESS;
}

uint32_t frm_feed(
        frm_ctx_t      *ctx,
        const uint8_t  *in_buf,
        const size_t    in_buf_len,
        frm_on_frame_t  on_frame,
        void           *user
){
        uint32_t frames = 0;

        if (ctx == NULL) {
                return 0;
        }
        if (in_buf == NULL) {
                if (in_buf_len > 0) {
                        ctx->debug.parser_err_nullarg++;
                }
                return 0;
        }

        for (size_t i = 0; i < in_buf_len; i++) {
                const uint8_t b = in_buf[i];

                if (ctx->parser.state == FRM_PARSER_STATE_DISCARD) {
                        if (b == FRM_BYTE_DELIMITER) {
                                ctx->parser.state = FRM_PARSER_STATE_RESET;
                                ctx->parser.encoded_len = 0;
                        }
                        continue;
                }

                if (b == FRM_BYTE_DELIMITER) {
                        uint16_t payload_len;

                        if (ctx->parser.encoded_len == 0) {
                                continue;                 /* empty frame / idle line */
                        }
                        if (frm_finish_frame(ctx, &payload_len) == FRM_PARSER_SUCCESS) {
                                frames++;
                                if (on_frame != NULL) {
                                        on_frame(&ctx->rx_raw[FRM_IDX_PAYLOAD], payload_len, user);
                                }
                        }
                        continue;                         /* keep scanning the buffer */
                }

                if (ctx->parser.encoded_len >= FRM_MAX_ENCODED_LEN) {
                        ctx->parser.state = FRM_PARSER_STATE_DISCARD;
                        ctx->parser.encoded_len = 0;
                        ctx->debug.parser_err_len++;
                        continue;
                }

                ctx->parser.encoded[ctx->parser.encoded_len++] = b;
                ctx->parser.state = FRM_PARSER_STATE_RECEIVING;
        }

        return frames;
}