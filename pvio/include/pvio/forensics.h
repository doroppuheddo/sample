#ifndef PVIO_FORENSICS_H
#define PVIO_FORENSICS_H

#include <any>
#include <mutex>
#include <pvio/pvio.h>

namespace pvio {

using point2i = vector<2, false, int>;
using color3b = vector<3, false, unsigned char>;

class ForensicsPainter {
  public:
    virtual ~ForensicsPainter() = default;
    virtual void set_image(const Image *image) = 0;
    virtual void point(const point2i &p, const color3b &c, int size = 1, int style = 0) {
    }
    virtual void line(const point2i &p1, const point2i &p2, const color3b &c, int thickness = 1) {
    }
};

class ForensicsSupport {
    struct VersionTag;

  public:
    enum ForensicsItem {
        RESERVED = 0,
        input_data_fps,
        input_real_fps,
        input_output_lag,
        feature_tracker_painter,
        sliding_window_track_painter,
        sliding_window_reprojection_painter,
        sliding_window_landmarks,
        sliding_window_planes,
        sliding_window_keyframe_poses,
        feature_tracker_time,
        bundle_adjustor_solve_time,
        bundle_adjustor_marginalization_time,
        ITEM_COUNT
    };

    ForensicsSupport(const VersionTag &tag);
    ~ForensicsSupport();

    static std::pair<std::any &, std::unique_lock<std::mutex>> get(ForensicsItem item);

  private:
    static ForensicsSupport &support();
    std::vector<std::pair<std::any, std::mutex>> storage;
};

} // namespace pvio

#if defined(PVIO_ENABLE_FORENSICS)
#define forensics(item, var) if constexpr (auto [var, var##_lock] = ::pvio::ForensicsSupport::get(::pvio::ForensicsSupport::item); true)
#else
#define forensics(item, var) if constexpr (std::any var; false)
#endif

#define critical_forensics(item, var) if constexpr (auto [var, var##_lock] = ::pvio::ForensicsSupport::get(::pvio::ForensicsSupport::item); true)

#endif // PVIO_FORENSICS_H
