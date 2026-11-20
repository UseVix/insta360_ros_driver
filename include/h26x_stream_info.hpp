// Minimal H.264 / H.265 bitstream inspection: NAL unit iteration and SPS parsing
// (picture size after cropping, and the nominal frame rate from H.264 VUI timing info).
// Header-only and allocation-light so it can run on every packet in the stream callback.
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace h26x {

enum class Codec { H264, H265 };

struct SpsInfo {
    int width = 0;
    int height = 0;
    double fps = 0.0;  // nominal frame rate from VUI timing info; 0 if not signalled (always 0 for H.265)
};

// Reads bits from an RBSP (emulation-prevention bytes already removed).
// Reading past the end yields zeros and sets overflow(), which callers treat as a parse failure.
class BitReader {
public:
    BitReader(const uint8_t* data, size_t size) : data_(data), size_(size) {}

    uint32_t u(int n) {
        uint32_t v = 0;
        for (int i = 0; i < n; ++i) {
            v <<= 1;
            if (pos_ >= size_ * 8) {
                overflow_ = true;
                continue;
            }
            v |= (data_[pos_ >> 3] >> (7 - (pos_ & 7))) & 1u;
            ++pos_;
        }
        return v;
    }
    void skip(size_t n) {
        pos_ += n;
        if (pos_ > size_ * 8) overflow_ = true;
    }
    uint32_t ue() {
        int zeros = 0;
        while (u(1) == 0) {
            if (overflow_ || ++zeros > 31) {
                overflow_ = true;
                return 0;
            }
        }
        if (zeros == 0) return 0;
        return ((1u << zeros) - 1u) + u(zeros);
    }
    int32_t se() {
        const uint32_t k = ue();
        return (k & 1u) ? static_cast<int32_t>((k + 1) / 2) : -static_cast<int32_t>(k / 2);
    }
    bool overflow() const { return overflow_; }

private:
    const uint8_t* data_;
    size_t size_;
    size_t pos_ = 0;
    bool overflow_ = false;
};

// Removes emulation-prevention bytes (00 00 03 -> 00 00).
inline std::vector<uint8_t> toRbsp(const uint8_t* nal, size_t size) {
    std::vector<uint8_t> out;
    out.reserve(size);
    int zeros = 0;
    for (size_t i = 0; i < size; ++i) {
        if (zeros >= 2 && nal[i] == 0x03) {
            zeros = 0;
            continue;
        }
        zeros = (nal[i] == 0) ? zeros + 1 : 0;
        out.push_back(nal[i]);
    }
    return out;
}

inline int nalType(Codec c, const uint8_t* nal, size_t size) {
    if (size < 1) return -1;
    return c == Codec::H264 ? (nal[0] & 0x1f) : ((nal[0] >> 1) & 0x3f);
}
inline bool isSps(Codec c, int type) { return c == Codec::H264 ? type == 7 : type == 33; }
inline bool isVcl(Codec c, int type) { return c == Codec::H264 ? (type >= 1 && type <= 5) : (type >= 0 && type <= 31); }
inline bool isKeyframe(Codec c, int type) { return c == Codec::H264 ? type == 5 : (type >= 16 && type <= 21); }

// Calls f(nal_ptr, nal_size) for every NAL unit in a packet. Annex-B (start codes) is the format the
// Insta360 SDK delivers; 4-byte big-endian length prefixes (AVCC/HVCC) are accepted as a fallback.
template <typename F>
void forEachNal(const uint8_t* d, size_t n, F&& f) {
    auto startCodeAt = [&](size_t i) -> size_t {  // length of the start code at i, or 0
        if (i + 3 <= n && d[i] == 0 && d[i + 1] == 0 && d[i + 2] == 1) return 3;
        if (i + 4 <= n && d[i] == 0 && d[i + 1] == 0 && d[i + 2] == 0 && d[i + 3] == 1) return 4;
        return 0;
    };
    if (startCodeAt(0) > 0) {
        size_t i = startCodeAt(0);
        size_t nal_start = i;
        while (i + 3 <= n) {
            size_t sc = (d[i] == 0 && d[i + 1] == 0) ? startCodeAt(i) : 0;
            if (sc > 0) {
                size_t end = i;
                while (end > nal_start && d[end - 1] == 0) --end;  // trailing_zero_8bits
                if (end > nal_start) f(d + nal_start, end - nal_start);
                i += sc;
                nal_start = i;
            } else {
                ++i;
            }
        }
        if (n > nal_start) f(d + nal_start, n - nal_start);
        return;
    }
    size_t i = 0;  // length-prefixed fallback
    while (i + 4 <= n) {
        const size_t len = (size_t(d[i]) << 24) | (size_t(d[i + 1]) << 16) | (size_t(d[i + 2]) << 8) | d[i + 3];
        i += 4;
        if (len == 0 || len > n - i) return;
        f(d + i, len);
        i += len;
    }
}

namespace detail {

inline void skipScalingList(BitReader& br, int size) {
    int last = 8, next = 8;
    for (int j = 0; j < size; ++j) {
        if (next != 0) next = (last + br.se() + 256) % 256;
        last = (next == 0) ? last : next;
    }
}

inline bool parseH264Sps(BitReader& br, SpsInfo& out) {
    const uint32_t profile_idc = br.u(8);
    br.skip(16);  // constraint flags + level_idc
    br.ue();      // seq_parameter_set_id
    uint32_t chroma_format_idc = 1;
    bool separate_colour_plane = false;
    switch (profile_idc) {
        case 100: case 110: case 122: case 244: case 44: case 83:
        case 86: case 118: case 128: case 138: case 139: case 134: case 135:
            chroma_format_idc = br.ue();
            if (chroma_format_idc == 3) separate_colour_plane = br.u(1);
            br.ue();  // bit_depth_luma_minus8
            br.ue();  // bit_depth_chroma_minus8
            br.u(1);  // qpprime_y_zero_transform_bypass_flag
            if (br.u(1)) {  // seq_scaling_matrix_present_flag
                const int lists = (chroma_format_idc != 3) ? 8 : 12;
                for (int i = 0; i < lists; ++i) {
                    if (br.u(1)) skipScalingList(br, i < 6 ? 16 : 64);
                }
            }
            break;
        default:
            break;
    }
    br.ue();  // log2_max_frame_num_minus4
    const uint32_t poc_type = br.ue();
    if (poc_type == 0) {
        br.ue();
    } else if (poc_type == 1) {
        br.u(1);
        br.se();
        br.se();
        const uint32_t cycle = br.ue();
        if (cycle > 255) return false;
        for (uint32_t i = 0; i < cycle; ++i) br.se();
    }
    br.ue();  // max_num_ref_frames
    br.u(1);  // gaps_in_frame_num_value_allowed_flag
    const uint32_t w_mbs = br.ue() + 1;
    const uint32_t h_map_units = br.ue() + 1;
    const uint32_t frame_mbs_only = br.u(1);
    if (!frame_mbs_only) br.u(1);  // mb_adaptive_frame_field_flag
    br.u(1);                       // direct_8x8_inference_flag
    uint32_t crop_l = 0, crop_r = 0, crop_t = 0, crop_b = 0;
    if (br.u(1)) {
        crop_l = br.ue();
        crop_r = br.ue();
        crop_t = br.ue();
        crop_b = br.ue();
    }
    if (br.overflow()) return false;

    const uint32_t array_type = separate_colour_plane ? 0 : chroma_format_idc;
    const uint32_t crop_x = (array_type == 0) ? 1 : (array_type == 3 ? 1 : 2);
    const uint32_t crop_y = ((array_type == 1) ? 2 : 1) * (2 - frame_mbs_only);
    const int64_t width = int64_t(w_mbs) * 16 - int64_t(crop_x) * (crop_l + crop_r);
    const int64_t height = int64_t(2 - frame_mbs_only) * h_map_units * 16 - int64_t(crop_y) * (crop_t + crop_b);
    if (width <= 0 || height <= 0 || width > 65536 || height > 65536) return false;
    out.width = static_cast<int>(width);
    out.height = static_cast<int>(height);
    out.fps = 0.0;

    if (br.u(1)) {  // vui_parameters_present_flag
        if (br.u(1)) {  // aspect_ratio_info_present_flag
            if (br.u(8) == 255) br.skip(32);  // Extended_SAR: sar_width, sar_height
        }
        if (br.u(1)) br.u(1);  // overscan_info_present_flag -> overscan_appropriate_flag
        if (br.u(1)) {         // video_signal_type_present_flag
            br.skip(4);        // video_format, video_full_range_flag
            if (br.u(1)) br.skip(24);  // colour_description_present_flag
        }
        if (br.u(1)) {  // chroma_loc_info_present_flag
            br.ue();
            br.ue();
        }
        if (br.u(1)) {  // timing_info_present_flag
            const uint32_t num_units_in_tick = br.u(32);
            const uint32_t time_scale = br.u(32);
            if (!br.overflow() && num_units_in_tick > 0) {
                out.fps = static_cast<double>(time_scale) / (2.0 * num_units_in_tick);
            }
        }
    }
    return true;
}

inline bool parseH265Sps(BitReader& br, SpsInfo& out) {
    br.skip(4);  // sps_video_parameter_set_id
    const uint32_t max_sub_layers_minus1 = br.u(3);
    br.u(1);     // sps_temporal_id_nesting_flag
    // profile_tier_level(1, max_sub_layers_minus1)
    br.skip(88);  // general profile space/tier/idc, compatibility flags, constraint flags
    br.skip(8);   // general_level_idc
    bool sub_profile[8] = {false}, sub_level[8] = {false};
    for (uint32_t i = 0; i < max_sub_layers_minus1; ++i) {
        sub_profile[i] = br.u(1);
        sub_level[i] = br.u(1);
    }
    if (max_sub_layers_minus1 > 0) {
        for (uint32_t i = max_sub_layers_minus1; i < 8; ++i) br.skip(2);
    }
    for (uint32_t i = 0; i < max_sub_layers_minus1; ++i) {
        if (sub_profile[i]) br.skip(88);
        if (sub_level[i]) br.skip(8);
    }
    br.ue();  // sps_seq_parameter_set_id
    const uint32_t chroma_format_idc = br.ue();
    if (chroma_format_idc == 3) br.u(1);  // separate_colour_plane_flag
    const uint32_t width = br.ue();
    const uint32_t height = br.ue();
    uint32_t l = 0, r = 0, t = 0, b = 0;
    if (br.u(1)) {  // conformance_window_flag
        l = br.ue();
        r = br.ue();
        t = br.ue();
        b = br.ue();
    }
    if (br.overflow()) return false;
    const uint32_t sub_w = (chroma_format_idc == 1 || chroma_format_idc == 2) ? 2 : 1;
    const uint32_t sub_h = (chroma_format_idc == 1) ? 2 : 1;
    const int64_t w = int64_t(width) - int64_t(sub_w) * (l + r);
    const int64_t h = int64_t(height) - int64_t(sub_h) * (t + b);
    if (w <= 0 || h <= 0 || w > 65536 || h > 65536) return false;
    out.width = static_cast<int>(w);
    out.height = static_cast<int>(h);
    out.fps = 0.0;  // H.265 VUI timing is not parsed
    return true;
}

}  // namespace detail

// Parses an SPS NAL unit (including its NAL header). Returns false if it is not an SPS or is malformed.
inline bool parseSps(Codec c, const uint8_t* nal, size_t size, SpsInfo& out) {
    const size_t header = (c == Codec::H264) ? 1 : 2;
    if (size <= header || !isSps(c, nalType(c, nal, size))) return false;
    const std::vector<uint8_t> rbsp = toRbsp(nal + header, size - header);
    BitReader br(rbsp.data(), rbsp.size());
    return c == Codec::H264 ? detail::parseH264Sps(br, out) : detail::parseH265Sps(br, out);
}

}  // namespace h26x
