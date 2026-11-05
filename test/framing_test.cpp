// GoogleTest unit tests for framing.c / framing.h (COBS framing)
//
// Wire format under test:   COBS( SIGNATURE | PAYLOAD | CRC16 [if enabled] ) | 0x00
//
// Suites:
//   FrmBuild         TX: frame layout against an independent COBS + CRC reference, error handling
//   FrmRoundTrip     ideal case (parameterized: CRC mode x payload size)
//   FrmPatterns      zero-heavy / 0xFF / COBS block-boundary payloads (CRC mode x pattern x size)
//   FrmStreaming     back-to-back frames, partial frames
//   FrmSplit/Chunked split / chunked buffers
//   FrmDesync        idle line, garbage, truncated frames, resync on the next delimiter
//   FrmSignature     wrong SIGNATURE bytes (with a valid CRC, so only the signature is wrong)
//   FrmCorruption    random garbage, delimiter-less floods, random body, single-bit sweep
//   FrmCobs          too-short frames, malformed COBS, encoded-length limit, buffer safety
//   FrmRobust        NULL arguments, corrupted parser state
//
// Run one test:   ./test_framing --gtest_filter=FrmCobs.*
// List tests:     ./test_framing --gtest_list_tests

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include "framing.h"

namespace {

using Bytes = std::vector<uint8_t>;

const Bytes kSig = { FRM_BYTE_SIGNATURE_1, FRM_BYTE_SIGNATURE_2 };

// ---------------------------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------------------------

// Arbitrary payload bytes (may contain zeros, which COBS must handle).
Bytes Payload(size_t n, unsigned seed)
{
    Bytes p(n);
    for (size_t i = 0; i < n; i++) {
        p[i] = static_cast<uint8_t>((seed + i * 7u) % 251u);
    }
    return p;
}

// Payload bytes in 1..255: no zeros at all (worst case for COBS overhead).
Bytes NonZeroPayload(size_t n, unsigned seed)
{
    Bytes p(n);
    for (size_t i = 0; i < n; i++) {
        p[i] = static_cast<uint8_t>(1u + (seed + i * 7u) % 255u);
    }
    return p;
}

Bytes Cat(std::initializer_list<Bytes> parts)
{
    Bytes out;
    for (const Bytes &p : parts) {
        out.insert(out.end(), p.begin(), p.end());
    }
    return out;
}

// Deterministic PRNG (xorshift32) so failures are reproducible.
struct Rng {
    uint32_t s;
    explicit Rng(uint32_t seed) : s(seed) {}
    uint8_t Byte()
    {
        s ^= s << 13;
        s ^= s >> 17;
        s ^= s << 5;
        return static_cast<uint8_t>(s >> 8);
    }
};

// Independent CRC-16 reference: bitwise, poly 0x8005, init 0, MSB first, no reflection,
// no final XOR (Dynamixel Protocol 2.0 flavour). Deliberately NOT table driven.
uint16_t Crc16Ref(const uint8_t *d, size_t n)
{
    uint16_t crc = 0;
    for (size_t i = 0; i < n; i++) {
        crc = static_cast<uint16_t>(crc ^ (static_cast<uint16_t>(d[i]) << 8));
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000u) ? static_cast<uint16_t>((crc << 1) ^ 0x8005u)
                                  : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

// Independent COBS reference encoder (no delimiter). Emits a new block only when more data
// follows a full 254-byte block.
Bytes RefCobsEncode(const Bytes &in)
{
    Bytes   out(1, 0);
    size_t  code_idx = 0;
    uint8_t code     = 1;
    for (uint8_t b : in) {
        if (code == 0xFF) {
            out[code_idx] = code;
            code_idx      = out.size();
            out.push_back(0);
            code = 1;
        }
        if (b == 0) {
            out[code_idx] = code;
            code_idx      = out.size();
            out.push_back(0);
            code = 1;
        } else {
            out.push_back(b);
            code++;
        }
    }
    out[code_idx] = code;
    return out;
}

// Independent COBS reference decoder. Accepts both encodings of a 254-byte run.
bool RefCobsDecode(const uint8_t *p, size_t n, Bytes &out)
{
    out.clear();
    size_t i = 0;
    while (i < n) {
        const uint8_t c = p[i++];
        if (c == 0 || i + (c - 1u) > n) {
            return false;
        }
        for (unsigned j = 1; j < c; j++) {
            out.push_back(p[i++]);
        }
        if (c < 0xFF && i < n) {
            out.push_back(0);
        }
    }
    return true;
}

// RAW frame = SIGNATURE | PAYLOAD | CRC16 (little endian, over SIGNATURE + PAYLOAD).
Bytes RefRaw(const Bytes &sig, const Bytes &payload, bool crc)
{
    Bytes raw = Cat({ sig, payload });
    if (crc) {
        const uint16_t c = Crc16Ref(raw.data(), raw.size());
        raw.push_back(static_cast<uint8_t>(c & 0xFF));
        raw.push_back(static_cast<uint8_t>(c >> 8));
    }
    return raw;
}

// Full wire frame from a RAW frame.
Bytes RefFrame(const Bytes &raw)
{
    Bytes f = RefCobsEncode(raw);
    f.push_back(FRM_BYTE_DELIMITER);
    return f;
}

class FramingTest : public ::testing::Test {
protected:
    void SetUp() override { Init(FRM_CRC_ENABLED); }

    void Init(frm_crc_mode_t mode)
    {
        frm_ctx_init(&ctx_, mode);
        got_.clear();
    }

    bool CrcOn() const { return ctx_.crc_mode == FRM_CRC_ENABLED; }

    static void Collect(const uint8_t *payload, uint16_t len, void *user)
    {
        static_cast<std::vector<Bytes> *>(user)->emplace_back(payload, payload + len);
    }

    uint32_t Feed(const Bytes &b) { return frm_feed(&ctx_, b.data(), b.size(), Collect, &got_); }

    uint32_t Feed(const uint8_t *p, size_t n) { return frm_feed(&ctx_, p, n, Collect, &got_); }

    // Build a full frame from `payload` using the current ctx's CRC mode.
    Bytes Build(const Bytes &payload)
    {
        auto f = std::make_unique<frm_frame_t>();
        EXPECT_EQ(frm_build(&ctx_, payload.data(), payload.size(), f.get()), FRM_BUILD_SUCCESS);
        return Bytes(f->buffer, f->buffer + f->frame_len);
    }

    // Hand-made frame (reference encoder) with an arbitrary SIGNATURE and a CRC that is valid for it.
    Bytes Frame(const Bytes &sig, const Bytes &payload) const
    {
        return RefFrame(RefRaw(sig, payload, CrcOn()));
    }

    uint32_t TotalErrors() const
    {
        return ctx_.debug.parser_err_crc + ctx_.debug.parser_err_len +
               ctx_.debug.parser_err_signature + ctx_.debug.parser_err_cobs;
    }

    frm_ctx_t          ctx_{};
    std::vector<Bytes> got_;
};

// One empty subclass per suite so TEST_F groups read nicely.
struct FrmBuild      : FramingTest {};
struct FrmStreaming  : FramingTest {};
struct FrmDesync     : FramingTest {};
struct FrmSignature  : FramingTest {};
struct FrmCorruption : FramingTest {};
struct FrmCobs       : FramingTest {};
struct FrmRobust     : FramingTest {};

// ---------------------------------------------------------------------------------------------
// TX: frm_build
// ---------------------------------------------------------------------------------------------

TEST_F(FrmBuild, LayoutWithCrc)
{
    const Bytes p = Payload(10, 1);
    const Bytes f = Build(p);

    // byte-exact against the reference encoder
    EXPECT_EQ(f, RefFrame(RefRaw(kSig, p, true)));
    // raw = 2 + 10 + 2 = 14 bytes -> 15 encoded + delimiter
    ASSERT_EQ(f.size(), static_cast<size_t>(FRM_OVERHEAD + 10 + 2));
    EXPECT_EQ(f.back(), FRM_BYTE_DELIMITER);

    // and the CRC inside is the independent one, little endian
    Bytes raw;
    ASSERT_TRUE(RefCobsDecode(f.data(), f.size() - 1, raw));
    const uint16_t crc = Crc16Ref(raw.data(), raw.size() - FRM_CRC_LEN);
    EXPECT_EQ(raw[raw.size() - 2], crc & 0xFF);
    EXPECT_EQ(raw[raw.size() - 1], crc >> 8);
    EXPECT_EQ(Bytes(raw.begin(), raw.begin() + FRM_SIGNATURE_LEN), kSig);
    EXPECT_EQ(Bytes(raw.begin() + FRM_IDX_PAYLOAD, raw.end() - FRM_CRC_LEN), p);
}

TEST_F(FrmBuild, LayoutWithoutCrc)
{
    Init(FRM_CRC_DISABLED);
    const Bytes p = Payload(10, 3);
    const Bytes f = Build(p);

    EXPECT_EQ(f, RefFrame(RefRaw(kSig, p, false)));
    ASSERT_EQ(f.size(), static_cast<size_t>(FRM_SIGNATURE_LEN + 10 + 2)); // no CRC bytes
}

TEST_F(FrmBuild, PayloadWithZeros)
{
    const Bytes p = { 0x00, 0x00, 0x11, 0x00, 0x22, 0x33, 0x00 };
    EXPECT_EQ(Build(p), RefFrame(RefRaw(kSig, p, true)));
    Init(FRM_CRC_DISABLED);
    EXPECT_EQ(Build(p), RefFrame(RefRaw(kSig, p, false)));
}

TEST_F(FrmBuild, EmptyPayload)
{
    Init(FRM_CRC_DISABLED);
    // raw = just the signature -> 03 31 EE 00
    EXPECT_EQ(Build({}), (Bytes{ 0x03, FRM_BYTE_SIGNATURE_1, FRM_BYTE_SIGNATURE_2, 0x00 }));
}

TEST_F(FrmBuild, NoZeroBytesBeforeDelimiter)
{
    const size_t big = std::min<size_t>(300, FRM_MAX_PAYLOAD_LEN);
    const Bytes payloads[] = {
        Bytes(big, 0x00), Bytes(big, 0xFF), Payload(big, 1), Payload(0, 1),
        NonZeroPayload(std::min<size_t>(254, FRM_MAX_PAYLOAD_LEN), 1), Bytes(1, 0x00)
    };
    for (const Bytes &p : payloads) {
        for (frm_crc_mode_t mode : { FRM_CRC_ENABLED, FRM_CRC_DISABLED }) {
            SCOPED_TRACE("payload size " + std::to_string(p.size()));
            Init(mode);
            const Bytes f = Build(p);
            ASSERT_FALSE(f.empty());
            EXPECT_EQ(f.back(), FRM_BYTE_DELIMITER);
            EXPECT_EQ(std::count(f.begin(), f.end(), 0x00), 1);
        }
    }
}

TEST_F(FrmBuild, CobsBlockBoundaries)
{
    for (size_t n = 246; n <= 262; n++) {
        if (n > FRM_MAX_PAYLOAD_LEN) continue;
        for (frm_crc_mode_t mode : { FRM_CRC_ENABLED, FRM_CRC_DISABLED }) {
            SCOPED_TRACE("n=" + std::to_string(n));
            Init(mode);
            const Bytes p = NonZeroPayload(n, 5);
            const Bytes f = Build(p);
            ASSERT_GE(f.size(), 2u);
            ASSERT_EQ(f.back(), FRM_BYTE_DELIMITER);
            Bytes raw;
            ASSERT_TRUE(RefCobsDecode(f.data(), f.size() - 1, raw));
            EXPECT_EQ(raw, RefRaw(kSig, p, CrcOn()));
        }
    }
}

TEST_F(FrmBuild, WorstCaseFitsBuffer)
{
    auto        fr = std::make_unique<frm_frame_t>();
    const Bytes p  = NonZeroPayload(FRM_MAX_PAYLOAD_LEN, 1);

    ASSERT_EQ(frm_build(&ctx_, p.data(), p.size(), fr.get()), FRM_BUILD_SUCCESS);
    EXPECT_LE(fr->frame_len, static_cast<uint32_t>(FRM_MAX_FRAME_LEN));
    EXPECT_GE(fr->frame_len, p.size() + FRM_OVERHEAD + 2);
    EXPECT_EQ(fr->payload_len, FRM_MAX_PAYLOAD_LEN);
}

TEST_F(FrmBuild, ErrorHandling)
{
    auto          f = std::make_unique<frm_frame_t>();
    const Bytes   p(FRM_MAX_PAYLOAD_LEN + 1, 0);

    EXPECT_EQ(frm_build(nullptr, p.data(), 8, f.get()), FRM_BUILD_ERROR_NULL_ARG);
    EXPECT_EQ(frm_build(&ctx_, p.data(), 8, nullptr), FRM_BUILD_ERROR_NULL_ARG);
    EXPECT_EQ(frm_build(&ctx_, nullptr, 8, f.get()), FRM_BUILD_ERROR_NULL_ARG);
    EXPECT_EQ(frm_build(&ctx_, p.data(), FRM_MAX_PAYLOAD_LEN + 1, f.get()),
              FRM_BUILD_ERROR_PAYLOAD_TOO_LONG);

    // NULL payload is fine when empty: raw = 4 bytes -> 5 encoded + delimiter
    EXPECT_EQ(frm_build(&ctx_, nullptr, 0, f.get()), FRM_BUILD_SUCCESS);
    EXPECT_EQ(f->frame_len, static_cast<uint32_t>(FRM_OVERHEAD + 2));
    EXPECT_EQ(f->payload_len, 0);
    EXPECT_EQ(f->buffer[f->frame_len - 1], FRM_BYTE_DELIMITER);
}

// ---------------------------------------------------------------------------------------------
// Ideal case (parameterized over CRC mode and payload size)
// ---------------------------------------------------------------------------------------------

class FrmRoundTrip : public FramingTest,
                     public ::testing::WithParamInterface<std::tuple<frm_crc_mode_t, size_t>> {};

TEST_P(FrmRoundTrip, DeliversPayload)
{
    const frm_crc_mode_t mode = std::get<0>(GetParam());
    const size_t         size = std::get<1>(GetParam());
    if (size > FRM_MAX_PAYLOAD_LEN) GTEST_SKIP();
    Init(mode);

    const Bytes p = Payload(size, 5);
    const Bytes f = Build(p);
    const size_t raw_len = FRM_SIGNATURE_LEN + size + (mode == FRM_CRC_ENABLED ? FRM_CRC_LEN : 0);
    EXPECT_GE(f.size(), raw_len + 2);                        // + code byte(s) + delimiter
    EXPECT_LE(f.size(), static_cast<size_t>(FRM_MAX_FRAME_LEN));

    EXPECT_EQ(Feed(f), 1u);
    ASSERT_EQ(got_.size(), 1u);
    EXPECT_EQ(got_[0], p);

    EXPECT_EQ(ctx_.debug.parser_frames_ok, 1u);
    EXPECT_EQ(TotalErrors(), 0u);
    EXPECT_EQ(ctx_.parser.state, FRM_PARSER_STATE_RESET);
    EXPECT_EQ(ctx_.parser.encoded_len, 0u);
}

static std::string RoundTripName(const ::testing::TestParamInfo<FrmRoundTrip::ParamType> &i)
{
    const bool   crc  = std::get<0>(i.param) == FRM_CRC_ENABLED;
    const size_t size = std::get<1>(i.param);
    return std::string(crc ? "Crc" : "NoCrc") + "_" +
           std::to_string(size) + "B_" + std::to_string(i.index);
}

INSTANTIATE_TEST_SUITE_P(
    ModesAndSizes, FrmRoundTrip,
    ::testing::Combine(::testing::Values(FRM_CRC_ENABLED, FRM_CRC_DISABLED),
                       ::testing::Values<size_t>(0, 1, 2, 16, 250, 251, 252, 253, 254, 255,
                                                 256, 300, FRM_MAX_PAYLOAD_LEN)),
    RoundTripName);

// ---------------------------------------------------------------------------------------------
// Payload patterns: zeros, 0xFF, alternating, around COBS block boundaries
// ---------------------------------------------------------------------------------------------

Bytes Pattern(int kind, size_t n)
{
    switch (kind) {
    case 0: return Bytes(n, 0x00);
    case 1: return NonZeroPayload(n, 3);
    case 2: return Bytes(n, 0xFF);
    case 3: {
        Bytes p(n);
        for (size_t i = 0; i < n; i++) {
            p[i] = (i % 2) ? 0xFF : 0x00;
        }
        return p;
    }
    default: return Payload(n, 7);
    }
}

class FrmPatterns : public FramingTest,
                    public ::testing::WithParamInterface<std::tuple<int, size_t>> {};

TEST_P(FrmPatterns, RoundTripBothModes)
{
    const size_t n = std::get<1>(GetParam());
    if (n > FRM_MAX_PAYLOAD_LEN) GTEST_SKIP();
    const Bytes p = Pattern(std::get<0>(GetParam()), std::get<1>(GetParam()));

    for (frm_crc_mode_t mode : { FRM_CRC_ENABLED, FRM_CRC_DISABLED }) {
        Init(mode);
        const Bytes f = Build(p);
        EXPECT_EQ(std::count(f.begin(), f.end(), 0x00), 1);   // only the delimiter
        EXPECT_EQ(Feed(f), 1u);
        ASSERT_EQ(got_.size(), 1u);
        EXPECT_EQ(got_[0], p);
        EXPECT_EQ(TotalErrors(), 0u);
    }
}

static std::string PatternName(const ::testing::TestParamInfo<FrmPatterns::ParamType> &i)
{
    static const char *const kNames[] = { "Zeros", "NonZero", "FF", "Alt00FF", "Mixed" };
    return std::string(kNames[std::get<0>(i.param)]) + "_" +
           std::to_string(std::get<1>(i.param)) + "B_" + std::to_string(i.index);
}

INSTANTIATE_TEST_SUITE_P(
    Patterns, FrmPatterns,
    ::testing::Combine(::testing::Range(0, 5),
                       ::testing::Values<size_t>(1, 2, 250, 252, 253, 254, 255,
                                                 256, 508, 1000, FRM_MAX_PAYLOAD_LEN)),
    PatternName);
// ---------------------------------------------------------------------------------------------
// Streaming
// ---------------------------------------------------------------------------------------------

TEST_F(FrmStreaming, BackToBackFrames)
{
    const Bytes pa = Payload(0, 1), pb = Payload(1, 2), pc = Payload(100, 3);
    const Bytes stream = Cat({ Build(pa), Build(pb), Build(pc) });

    EXPECT_EQ(Feed(stream), 3u);
    ASSERT_EQ(got_.size(), 3u);
    EXPECT_EQ(got_[0], pa);
    EXPECT_EQ(got_[1], pb);
    EXPECT_EQ(got_[2], pc);
    EXPECT_EQ(ctx_.debug.parser_frames_ok, 3u);
}

TEST_F(FrmStreaming, ExtraDelimitersBetweenFramesAreIdle)
{
    const Bytes pa = Payload(8, 1), pb = Payload(8, 2);
    EXPECT_EQ(Feed(Cat({ Build(pa), Bytes(5, 0x00), Build(pb), Bytes(2, 0x00) })), 2u);
    ASSERT_EQ(got_.size(), 2u);
    EXPECT_EQ(got_[0], pa);
    EXPECT_EQ(got_[1], pb);
    EXPECT_EQ(TotalErrors(), 0u);
}

TEST_F(FrmStreaming, PartialFrameKeepsState)
{
    const Bytes f = Build(Payload(8, 1));

    EXPECT_EQ(Feed(f.data(), f.size() - 1), 0u);
    EXPECT_EQ(ctx_.parser.state, FRM_PARSER_STATE_RECEIVING);
    EXPECT_EQ(ctx_.parser.encoded_len, f.size() - 1);
}

// ---------------------------------------------------------------------------------------------
// Split / streaming buffers
// ---------------------------------------------------------------------------------------------

constexpr size_t kSplitFrameLen = FRM_OVERHEAD + 20 + 2; // raw 24 -> 25 encoded + delimiter

class FrmSplit : public FramingTest, public ::testing::WithParamInterface<size_t> {};

TEST_P(FrmSplit, FrameSplitAcrossTwoFeeds)
{
    const size_t split = GetParam();
    const Bytes  p     = Payload(20, 4);
    const Bytes  f     = Build(p);
    ASSERT_EQ(f.size(), kSplitFrameLen);
    ASSERT_LT(split, f.size());

    EXPECT_EQ(Feed(f.data(), split), 0u);
    EXPECT_EQ(Feed(f.data() + split, f.size() - split), 1u);
    ASSERT_EQ(got_.size(), 1u);
    EXPECT_EQ(got_[0], p);
}

INSTANTIATE_TEST_SUITE_P(EverySplitPoint, FrmSplit, ::testing::Range<size_t>(1, kSplitFrameLen));

class FrmChunked : public FramingTest, public ::testing::WithParamInterface<size_t> {};

TEST_P(FrmChunked, ThreeFramesInFixedChunks)
{
    const size_t chunk = GetParam();
    const Bytes  pa = Payload(17, 1), pb = Payload(40, 2), pc = Payload(0, 3);
    const Bytes  stream = Cat({ Build(pa), Build(pb), Build(pc) });

    uint32_t total = 0;
    for (size_t pos = 0; pos < stream.size(); pos += chunk) {
        total += Feed(stream.data() + pos, std::min(chunk, stream.size() - pos));
    }
    EXPECT_EQ(total, 3u);
    ASSERT_EQ(got_.size(), 3u);
    EXPECT_EQ(got_[0], pa);
    EXPECT_EQ(got_[1], pb);
    EXPECT_EQ(got_[2], pc);
}

INSTANTIATE_TEST_SUITE_P(ChunkSizes, FrmChunked, ::testing::Values<size_t>(1, 2, 3, 5, 7, 11, 64));

// ---------------------------------------------------------------------------------------------
// Desynchronization
// ---------------------------------------------------------------------------------------------

TEST_F(FrmDesync, IdleLineIsNotAnError)
{
    EXPECT_EQ(Feed(Bytes(1000, 0x00)), 0u);
    EXPECT_EQ(TotalErrors(), 0u);
    EXPECT_EQ(ctx_.parser.state, FRM_PARSER_STATE_RESET);
}

TEST_F(FrmDesync, LeadingDelimiters)
{
    const Bytes p = Payload(12, 1);
    for (size_t n = 1; n <= 10; n++) {
        SCOPED_TRACE("n=" + std::to_string(n));
        Init(FRM_CRC_ENABLED);
        EXPECT_EQ(Feed(Cat({ Bytes(n, 0x00), Build(p) })), 1u);
        ASSERT_EQ(got_.size(), 1u);
        EXPECT_EQ(got_[0], p);
        EXPECT_EQ(TotalErrors(), 0u);
    }
}

TEST_F(FrmDesync, GarbageTerminatedByDelimiterIsOneError)
{
    // "11" claims 16 more bytes but only 4 follow: rejected at the delimiter, next frame is fine.
    const Bytes noise = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x00 };
    const Bytes p     = Payload(12, 1);

    EXPECT_EQ(Feed(Cat({ noise, Build(p) })), 1u);
    ASSERT_EQ(got_.size(), 1u);
    EXPECT_EQ(got_[0], p);
    EXPECT_EQ(TotalErrors(), 1u);
}

TEST_F(FrmDesync, GarbageWithoutDelimiterCostsOneFrame)
{
    // No delimiter between the garbage and frame A: they merge into one bad frame (A is lost),
    // and frame B is received intact.
    const Bytes noise = { 0x11, 0x22, 0x33, 0x44, 0x55 };
    const Bytes pb    = Payload(12, 2);

    EXPECT_EQ(Feed(Cat({ noise, Build(Payload(12, 1)), Build(pb) })), 1u);
    ASSERT_EQ(got_.size(), 1u);
    EXPECT_EQ(got_[0], pb);
    EXPECT_EQ(TotalErrors(), 1u);
}

TEST_F(FrmDesync, NoiseBetweenFrames)
{
    const Bytes pa = Payload(9, 1), pb = Payload(21, 2), pc = Payload(5, 3);

    // Random noise (zeros allowed) always closed by a delimiter so it cannot swallow a frame.
    Rng   rng(0x1234ABCDu);
    Bytes noise;
    for (int i = 0; i < 37; i++) {
        noise.push_back(rng.Byte());
    }
    noise.push_back(0x00);

    EXPECT_EQ(Feed(Cat({ Build(pa), noise, Build(pb), noise, Build(pc) })), 3u);
    ASSERT_EQ(got_.size(), 3u);
    EXPECT_EQ(got_[0], pa);
    EXPECT_EQ(got_[1], pb);
    EXPECT_EQ(got_[2], pc);
    EXPECT_GE(TotalErrors(), 1u);
}

TEST_F(FrmDesync, TruncatedFrameThenReset)
{
    const Bytes pb = Payload(20, 2);
    const Bytes fa = Build(Payload(20, 1));
    const Bytes fb = Build(pb);

    // half of A arrives, then the link drops
    EXPECT_EQ(Feed(fa.data(), fa.size() / 2), 0u);
    EXPECT_EQ(ctx_.parser.state, FRM_PARSER_STATE_RECEIVING);

    frm_ctx_reset(&ctx_);
    EXPECT_EQ(ctx_.parser.state, FRM_PARSER_STATE_RESET);
    EXPECT_EQ(ctx_.parser.encoded_len, 0u);

    // B comes through cleanly and counters survive the reset
    EXPECT_EQ(Feed(fb), 1u);
    ASSERT_EQ(got_.size(), 1u);
    EXPECT_EQ(got_[0], pb);
    EXPECT_EQ(ctx_.debug.parser_frames_ok, 1u);
}

TEST_F(FrmDesync, TruncatedFrameSelfRecovers)
{
    // No reset: half of A, then a stream of good frames. The first B is merged with the half of A
    // and lost; the parser must resynchronize on the next delimiter and deliver the rest.
    const Bytes pb = Payload(20, 2);
    const Bytes fa = Build(Payload(20, 1));
    const Bytes fb = Build(pb);

    Bytes stream(fa.begin(), fa.begin() + fa.size() / 2);
    for (int i = 0; i < 4; i++) {
        stream.insert(stream.end(), fb.begin(), fb.end());
    }
    Feed(stream);

    ASSERT_EQ(got_.size(), 3u);
    EXPECT_EQ(got_.back(), pb);
    EXPECT_EQ(TotalErrors(), 1u);
    EXPECT_EQ(ctx_.parser.state, FRM_PARSER_STATE_RESET);
}

// ---------------------------------------------------------------------------------------------
// Wrong SIGNATURE (valid CRC, so the signature check is what rejects the frame)
// ---------------------------------------------------------------------------------------------

class FrmBadSignature : public FramingTest, public ::testing::WithParamInterface<Bytes> {};

TEST_P(FrmBadSignature, FeedReportsSignatureError)
{
    for (frm_crc_mode_t mode : { FRM_CRC_ENABLED, FRM_CRC_DISABLED }) {
        Init(mode);
        const Bytes bad = Frame(GetParam(), Payload(10, 1));

        EXPECT_EQ(Feed(bad), 0u);
        EXPECT_EQ(ctx_.debug.parser_err_signature, 1u);
        EXPECT_EQ(TotalErrors(), 1u);
        EXPECT_EQ(ctx_.parser.state, FRM_PARSER_STATE_RESET);
    }
}

TEST_P(FrmBadSignature, FeedSkipsBadFrameAndGetsNextFrame)
{
    const Bytes p = Payload(10, 1);
    EXPECT_EQ(Feed(Cat({ Frame(GetParam(), p), Build(p) })), 1u);
    ASSERT_EQ(got_.size(), 1u);
    EXPECT_EQ(got_[0], p);
    EXPECT_EQ(ctx_.debug.parser_err_signature, 1u);
    EXPECT_EQ(TotalErrors(), 1u);
}

INSTANTIATE_TEST_SUITE_P(
    BadSignatures, FrmBadSignature,
    ::testing::Values(Bytes{ 0x99, FRM_BYTE_SIGNATURE_2 },                    // wrong 1st byte
                      Bytes{ FRM_BYTE_SIGNATURE_1, 0x00 },                    // wrong 2nd byte
                      Bytes{ FRM_BYTE_SIGNATURE_2, FRM_BYTE_SIGNATURE_1 },    // swapped
                      Bytes{ 0x00, 0x00 },                                    // zeros
                      Bytes{ 0xFF, 0xFA }));                                  // old sync bytes

// ---------------------------------------------------------------------------------------------
// Total corruption
// ---------------------------------------------------------------------------------------------

class FrmRandomGarbage
    : public FramingTest,
      public ::testing::WithParamInterface<std::tuple<frm_crc_mode_t, uint32_t>> {};

TEST_P(FrmRandomGarbage, NoFramesAndParserSurvives)
{
    Init(std::get<0>(GetParam()));
    Rng rng(std::get<1>(GetParam()));

    Bytes noise(50000);
    for (uint8_t &b : noise) {
        b = rng.Byte();
    }

    EXPECT_EQ(Feed(noise), 0u);
    EXPECT_EQ(ctx_.debug.parser_frames_ok, 0u);

    // Without any reset, the next delimiter resynchronizes and a good frame gets through
    const Bytes p = Payload(24, 8);
    EXPECT_EQ(Feed(Cat({ Bytes(1, 0x00), Build(p) })), 1u);
    ASSERT_EQ(got_.size(), 1u);
    EXPECT_EQ(got_[0], p);
}

std::string GarbageName(const ::testing::TestParamInfo<FrmRandomGarbage::ParamType> &i)
{
    return std::string(std::get<0>(i.param) == FRM_CRC_ENABLED ? "Crc" : "NoCrc") + "_seed" +
           std::to_string(i.index % 5);
}

INSTANTIATE_TEST_SUITE_P(
    Seeds, FrmRandomGarbage,
    ::testing::Combine(::testing::Values(FRM_CRC_ENABLED, FRM_CRC_DISABLED),
                       ::testing::Values<uint32_t>(1u, 0xDEADBEEFu, 0xC0FFEEu, 0x12345678u, 0xA5A5A5A5u)),
    GarbageName);

TEST_F(FrmCorruption, NonZeroFloodWithoutDelimiter)
{
    // 50000 bytes with no delimiter: reported once as too long, then dropped until the delimiter.
    Bytes flood(50000, 0x55);

    EXPECT_EQ(Feed(flood), 0u);
    EXPECT_EQ(ctx_.debug.parser_err_len, 1u);
    EXPECT_EQ(TotalErrors(), 1u);
    EXPECT_EQ(ctx_.parser.state, FRM_PARSER_STATE_DISCARD);

    const Bytes p = Payload(10, 5);
    EXPECT_EQ(Feed(Cat({ Bytes(1, 0x00), Build(p) })), 1u);
    ASSERT_EQ(got_.size(), 1u);
    EXPECT_EQ(got_[0], p);
    EXPECT_EQ(TotalErrors(), 1u);
}

TEST_F(FrmCorruption, RandomBodyBehindValidSignature)
{
    // Well-formed COBS, valid SIGNATURE, random body and random CRC: the CRC must reject it
    Rng   rng(0xFEEDFACEu);
    Bytes raw = kSig;
    for (int i = 0; i < 64; i++) {
        raw.push_back(rng.Byte());
    }

    EXPECT_EQ(Feed(RefFrame(raw)), 0u);
    EXPECT_EQ(ctx_.debug.parser_err_crc, 1u);
    EXPECT_EQ(ctx_.parser.state, FRM_PARSER_STATE_RESET);

    // a good frame right after is fine
    const Bytes p = Payload(10, 5);
    EXPECT_EQ(Feed(Build(p)), 1u);
    ASSERT_EQ(got_.size(), 1u);
    EXPECT_EQ(got_[0], p);
}

// For every bit of every byte of frame A (delimiter included): flip it, send A', one extra
// delimiter to close whatever A' turned into, then a good frame B. A' must never be delivered;
// B must always be delivered.
constexpr size_t kFlipFrameLen = FRM_OVERHEAD + 16 + 2;

class FrmBitFlip : public FramingTest, public ::testing::WithParamInterface<size_t> {};

TEST_P(FrmBitFlip, NeverDeliversCorruptedFrameAndAlwaysRecovers)
{
    const size_t pos = GetParam();
    const Bytes  pb  = Payload(16, 2);

    for (int bit = 0; bit < 8; bit++) {
        SCOPED_TRACE("bit=" + std::to_string(bit));
        Init(FRM_CRC_ENABLED);

        Bytes fa = Build(Payload(16, 1));
        ASSERT_EQ(fa.size(), kFlipFrameLen);
        fa[pos] ^= static_cast<uint8_t>(1u << bit);

        const Bytes stream = Cat({ fa, Bytes(1, 0x00), Build(pb) });
        EXPECT_EQ(Feed(stream), 1u);
        ASSERT_EQ(got_.size(), 1u);
        EXPECT_EQ(got_[0], pb);
    }
}

INSTANTIATE_TEST_SUITE_P(EveryByte, FrmBitFlip, ::testing::Range<size_t>(0, kFlipFrameLen),
                         [](const ::testing::TestParamInfo<size_t> &i) {
                             return "byte" + std::to_string(i.param);
                         });

// ---------------------------------------------------------------------------------------------
// COBS-specific behaviour
// ---------------------------------------------------------------------------------------------

TEST_F(FrmCobs, TooShortFramesWithCrc)
{
    // CRC mode needs at least SIGNATURE + CRC = 4 raw bytes
    const Bytes raws[] = { { 0x31 }, { 0x31, 0xEE }, { 0x31, 0xEE, 0x00 } };
    for (const Bytes &raw : raws) {
        SCOPED_TRACE("raw size " + std::to_string(raw.size()));
        Init(FRM_CRC_ENABLED);

        EXPECT_EQ(Feed(RefFrame(raw)), 0u);
        EXPECT_EQ(ctx_.debug.parser_err_len, 1u);
        EXPECT_EQ(TotalErrors(), 1u);
        EXPECT_EQ(ctx_.parser.state, FRM_PARSER_STATE_RESET);
    }
}

TEST_F(FrmCobs, TooShortFramesWithoutCrc)
{
    Init(FRM_CRC_DISABLED);

    EXPECT_EQ(Feed(RefFrame({ 0x31 })), 0u);
    EXPECT_EQ(ctx_.debug.parser_err_len, 1u);

    // SIGNATURE only is a valid, empty frame
    EXPECT_EQ(Feed(RefFrame(kSig)), 1u);
    ASSERT_EQ(got_.size(), 1u);
    EXPECT_TRUE(got_[0].empty());
}

TEST_F(FrmCobs, MalformedCobsIsRejectedAndStreamRecovers)
{
    // The code byte promises more data than the frame holds
    const Bytes bad[] = { { 0x05, 0x01, 0x00 }, { 0xFF, 0x01, 0x02, 0x00 }, { 0x02, 0x31, 0x09, 0x00 } };
    const Bytes p     = Payload(10, 1);

    for (const Bytes &b : bad) {
        Init(FRM_CRC_ENABLED);
        EXPECT_EQ(Feed(Cat({ b, Build(p) })), 1u);
        ASSERT_EQ(got_.size(), 1u);
        EXPECT_EQ(got_[0], p);
        EXPECT_EQ(TotalErrors(), 1u);
    }

    Init(FRM_CRC_ENABLED);
    EXPECT_EQ(Feed(bad[0]), 0u);
    EXPECT_EQ(ctx_.debug.parser_err_cobs, 1u);
}

TEST_F(FrmCobs, EncodedLengthLimit)
{
    const Bytes body(FRM_MAX_ENCODED_LEN, 0x55);

    // Exactly the maximum encoded length is still accepted (parser waits for the delimiter)
    EXPECT_EQ(Feed(body), 0u);
    EXPECT_EQ(ctx_.parser.encoded_len, static_cast<uint32_t>(FRM_MAX_ENCODED_LEN));
    EXPECT_EQ(ctx_.parser.state, FRM_PARSER_STATE_RECEIVING);
    EXPECT_EQ(TotalErrors(), 0u);

    // One more byte is rejected immediately
    EXPECT_EQ(Feed(Bytes{ 0x55 }), 0u);
    EXPECT_EQ(ctx_.debug.parser_err_len, 1u);
    EXPECT_EQ(ctx_.parser.state, FRM_PARSER_STATE_DISCARD);
    EXPECT_EQ(ctx_.parser.encoded_len, 0u);

    // Everything up to the next delimiter is dropped silently
    EXPECT_EQ(Feed(Bytes(100, 0x66)), 0u);
    EXPECT_EQ(ctx_.debug.parser_err_len, 1u);
    EXPECT_EQ(ctx_.parser.state, FRM_PARSER_STATE_DISCARD);

    // The delimiter resynchronizes
    EXPECT_EQ(Feed(Bytes{ 0x00 }), 0u);
    EXPECT_EQ(ctx_.parser.state, FRM_PARSER_STATE_RESET);

    const Bytes p = Payload(10, 4);
    EXPECT_EQ(Feed(Build(p)), 1u);
    ASSERT_EQ(got_.size(), 1u);
    EXPECT_EQ(got_[0], p);
}

TEST_F(FrmCobs, OverlongFramesDoNotOverflowBuffer)
{
    // Several over-long frames back to back; a stale length must never let the parser write past
    // parser.encoded (intra-struct, so ASan can't see it: the state/counter checks below catch it).
    Bytes frame(FRM_MAX_ENCODED_LEN + 10, 0x55);
    frame.push_back(0x00);

    for (int i = 0; i < 4; i++) {
        Feed(frame);
    }
    EXPECT_EQ(ctx_.debug.parser_err_len, 4u);
    EXPECT_EQ(ctx_.debug.parser_frames_ok, 0u);
    EXPECT_EQ(ctx_.parser.state, FRM_PARSER_STATE_RESET);

    const Bytes p = Payload(10, 6);
    EXPECT_EQ(Feed(Build(p)), 1u);
}

TEST_F(FrmCobs, MaxSizeFrameAtExactEncodedLimit)
{
    // A real maximum-size frame (no zeros anywhere: worst-case COBS overhead) is accepted
    // and its encoded length never exceeds the documented bound.
    for (frm_crc_mode_t mode : { FRM_CRC_ENABLED, FRM_CRC_DISABLED }) {
        Init(mode);
        const Bytes p = NonZeroPayload(FRM_MAX_PAYLOAD_LEN, 2);
        const Bytes f = Build(p);
        EXPECT_LE(f.size() - 1, static_cast<size_t>(FRM_MAX_ENCODED_LEN));
        EXPECT_EQ(Feed(f), 1u);
        ASSERT_EQ(got_.size(), 1u);
        EXPECT_EQ(got_[0], p);
        EXPECT_EQ(TotalErrors(), 0u);
    }
}

// ---------------------------------------------------------------------------------------------
// Robustness
// ---------------------------------------------------------------------------------------------

TEST_F(FrmRobust, NullAndEmptyArguments)
{
    uint8_t b = 0;

    EXPECT_EQ(frm_feed(nullptr, &b, 1, Collect, &got_), 0u);
    EXPECT_EQ(frm_feed(&ctx_, nullptr, 1, Collect, &got_), 0u);
    EXPECT_EQ(ctx_.debug.parser_err_nullarg, 1u);

    EXPECT_EQ(frm_feed(&ctx_, nullptr, 0, Collect, &got_), 0u);
    EXPECT_EQ(ctx_.debug.parser_err_nullarg, 1u); // empty NULL buffer is not an error
    EXPECT_EQ(frm_feed(&ctx_, &b, 0, Collect, &got_), 0u); // empty non-NULL buffer

    // NULL callback: frames are parsed and counted, nothing crashes
    const Bytes f = Build(Payload(5, 1));
    EXPECT_EQ(frm_feed(&ctx_, f.data(), f.size(), nullptr, nullptr), 1u);

    // NULL-safe init/reset
    frm_ctx_init(nullptr, FRM_CRC_ENABLED);
    frm_ctx_reset(nullptr);
}

TEST_F(FrmRobust, CorruptedParserStateRecovers)
{
    const Bytes p = Payload(8, 1);
    const Bytes f = Build(p);

    // Garbage state value, e.g. a memory glitch. Written bytewise: converting 99 to the enum type
    // directly would itself be undefined behaviour in C++.
    const int garbage = 99;
    static_assert(sizeof(ctx_.parser.state) <= sizeof(garbage), "unexpected enum size");
    std::memcpy(&ctx_.parser.state, &garbage, sizeof(ctx_.parser.state));
    EXPECT_EQ(Feed(f), 1u);
    ASSERT_EQ(got_.size(), 1u);
    EXPECT_EQ(got_[0], p);

    Init(FRM_CRC_ENABLED);
    ctx_.parser.state = FRM_PARSER_STATE_RESET;
    EXPECT_EQ(Feed(f), 1u);
    ASSERT_EQ(got_.size(), 1u);
    EXPECT_EQ(got_[0], p);
}

TEST_F(FrmRobust, CorruptedEncodedLengthNeverReadsPastBuffer)
{
    const Bytes p = Payload(8, 1);
    const Bytes f = Build(p);

    // Glitched length counter, then a delimiter: must be rejected, not decoded
    ctx_.parser.state       = FRM_PARSER_STATE_RECEIVING;
    ctx_.parser.encoded_len = 0xFFFFFFFFu;
    EXPECT_EQ(Feed(Bytes(1, 0x00)), 0u);
    EXPECT_EQ(ctx_.debug.parser_err_len, 1u);
    EXPECT_EQ(ctx_.parser.state, FRM_PARSER_STATE_RESET);
    EXPECT_EQ(ctx_.parser.encoded_len, 0u);

    // Glitched length counter, then data: dropped until the delimiter, then back to normal
    ctx_.parser.state       = FRM_PARSER_STATE_RECEIVING;
    ctx_.parser.encoded_len = 0xFFFFFFFFu;
    EXPECT_EQ(Feed(f), 0u);
    EXPECT_EQ(Feed(f), 1u);
    ASSERT_EQ(got_.size(), 1u);
    EXPECT_EQ(got_[0], p);
}

TEST_F(FrmBuild, CrcMatchesReferenceAcrossAllTableEntries)
{
    // Drive every possible high byte through the accumulator so all 256 LUT
    // entries are exercised. A single-byte input with value i forces index i
    // on the first (and only) iteration: index = (0 >> 8) ^ i = i.
    for (int i = 0; i < 256; i++) {
        const uint8_t  byte = static_cast<uint8_t>(i);
        frm_frame_t    f{};
        ASSERT_EQ(frm_build(&ctx_, &byte, 1, &f), FRM_BUILD_SUCCESS);

        // Decode the frame and extract the CRC the library wrote
        Bytes raw;
        ASSERT_TRUE(RefCobsDecode(f.buffer, f.frame_len - 1, raw));
        ASSERT_GE(raw.size(), static_cast<size_t>(FRM_OVERHEAD));

        const size_t   body_len = raw.size() - FRM_CRC_LEN;
        const uint16_t lib_crc  = static_cast<uint16_t>(raw[body_len] |
                                  (static_cast<uint16_t>(raw[body_len + 1]) << 8));
        const uint16_t ref_crc  = Crc16Ref(raw.data(), body_len);

        SCOPED_TRACE("byte=" + std::to_string(i));
        EXPECT_EQ(lib_crc, ref_crc);
    }
}

TEST_F(FrmCobs, DecodedOutputLargerThanRawBufferIsRejected)
{
    // Each 0x01 code byte decodes to one zero, so N codes decode to N-1 bytes.
    // That only exceeds rx_raw when FRM_MAX_RAW_LEN > 254.
    if (FRM_MAX_ENCODED_LEN - 1 <= FRM_MAX_RAW_LEN)
        GTEST_SKIP() << "decoded output cannot exceed rx_raw at this payload size";

    Bytes f(FRM_MAX_ENCODED_LEN, 0x01);
    f.push_back(0x00);
    EXPECT_EQ(Feed(f), 0u);
    EXPECT_EQ(ctx_.debug.parser_err_cobs, 1u);  // a mutant overwrites debug, then bumps err_signature
    EXPECT_EQ(TotalErrors(), 1u);
}

TEST_F(FrmCobs, EncodedLengthAtLimitReachesDecoder)
{
    Bytes f(FRM_MAX_ENCODED_LEN, 0x55);
    f.push_back(0x00);
    EXPECT_EQ(Feed(f), 0u);
    EXPECT_EQ(ctx_.debug.parser_err_len, 0u);   // must reach the decoder, not be rejected as too long
    EXPECT_EQ(TotalErrors(), 1u);
}

TEST_F(FrmStreaming, ParserStateAfterFrameAndReset)
{
    const Bytes p = Payload(10, 1);
    const Bytes f = Build(p);

    // A complete frame is delivered with the right length and leaves nothing buffered
    ASSERT_EQ(Feed(f), 1u);
    ASSERT_EQ(got_.size(), 1u);
    EXPECT_EQ(got_[0].size(), p.size());
    EXPECT_EQ(ctx_.parser.state, FRM_PARSER_STATE_RESET);
    EXPECT_EQ(ctx_.parser.encoded_len, 0u);

    // A partial frame is buffered; reset drops it but keeps CRC mode and counters
    ASSERT_EQ(Feed(f.data(), f.size() - 1), 0u);
    EXPECT_EQ(ctx_.parser.encoded_len, f.size() - 1);

    frm_ctx_reset(&ctx_);
    EXPECT_EQ(ctx_.parser.state, FRM_PARSER_STATE_RESET);
    EXPECT_EQ(ctx_.parser.encoded_len, 0u);
    EXPECT_EQ(ctx_.crc_mode, FRM_CRC_ENABLED);
    EXPECT_EQ(ctx_.debug.parser_frames_ok, 1u);
}

} // namespace