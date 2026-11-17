#include "SampleConfigFactory.hpp"

#include <libslic3r/EngineErrorCodes.hpp>

using namespace Slic3r::sla;

namespace {
// One pass over the constraints. Returns true when nothing had to be changed.
// It moves values toward the constraints: clamping to a bound, swapping a
// min/max pair that is out of order, or halving a min that equals its max.
// Such changes are expected once density scales the distances, not an error.
bool verify_once(SampleConfig &cfg) {
    auto verify_max = [](coord_t &c, coord_t max) {
        if (c > max) {
            c = max;
            return false;
        }
        return true;
    };
    auto verify_min = [](coord_t &c, coord_t min) {
        if (c < min) {
            c = min;
            return false;
        }
        return true;
    };
    auto verify_min_max = [](coord_t &min, coord_t &max) {
        // min must be smaller than max
        if (min > max) {
            std::swap(min, max);
            return false;
        } else if (min == max) {
            min /= 2; // cut in half
            return false;
        }
        return true;
    };
    bool res = true;
    res &= verify_min_max(cfg.max_length_for_one_support_point, cfg.max_length_for_two_support_points);        
    res &= verify_min_max(cfg.thick_min_width, cfg.thin_max_width); // check histeresis
    res &= verify_max(cfg.max_length_for_one_support_point,
        2 * cfg.thin_max_distance +
        2 * cfg.head_radius +
        2 * cfg.minimal_distance_from_outline);
    res &= verify_min(cfg.max_length_for_one_support_point,
        2 * cfg.head_radius + 2 * cfg.minimal_distance_from_outline);
    res &= verify_max(cfg.max_length_for_two_support_points,
        2 * cfg.thin_max_distance + 
        2 * 2 * cfg.head_radius +
        2 * cfg.minimal_distance_from_outline);
    res &= verify_min(cfg.thin_max_width, 
        2 * cfg.head_radius + 2 * cfg.minimal_distance_from_outline);
    res &= verify_max(cfg.thin_max_width,
        2 * cfg.thin_max_distance + 2 * cfg.head_radius);
    return res;
}

// Passes allowed after the first one. Valid configurations settle in a few
// passes; this only stops contradictory ones (e.g. thin_max_distance below
// minimal_distance_from_outline) from looping forever.
constexpr int max_extra_verify_passes = 20;
} // namespace

bool SampleConfigFactory::verify(SampleConfig &cfg) {
    if (verify_once(cfg))
        return true;
    // Repeat until a pass changes nothing. Such a pass leaves the config as it
    // is, so the result does not depend on how many passes it took.
    for (int i = 0; i < max_extra_verify_passes; ++i)
        if (verify_once(cfg))
            return false;
    throw Slic3r::CodedRuntimeError(Slic3r::make_engine_error(
        Slic3r::EngineErrorCode::SUPPORT_POINT_SAMPLING_FAILED,
        "SampleConfig convergence failed: cyclic geometric constraint"));
}

SampleConfig SampleConfigFactory::create(float support_head_diameter_in_mm) {
    SampleConfig result;
    result.head_radius = static_cast<coord_t>(scale_(support_head_diameter_in_mm/2));
    
    assert(result.minimal_distance_from_outline < result.maximal_distance_from_outline);

    // https://cfl.prusa3d.com/display/SLA/Single+Supporty
    // head 0.4mm cca 1.65 mm
    // head 0.5mm cca 1.85 mm
    // This values are used for solvig equation(to find 2.9 and 1.3)
    double head_area = M_PI * sqr(support_head_diameter_in_mm / 2); // Pi r^2
    result.max_length_for_one_support_point = static_cast<coord_t>(scale_(head_area * 2.9 + 1.3));

    // https://cfl.prusa3d.com/display/SLA/Double+Supports+-+Rectangles
    // head 0.4mm cca 6.5 mm
    // Use linear dependency to max_length_for_one_support_point
    result.max_length_for_two_support_points = 
        static_cast<coord_t>(result.max_length_for_one_support_point * 3.9);

    // https://cfl.prusa3d.com/display/SLA/Double+Supports+-+Squares
    // head 0.4mm cca (4.168 to 4.442) => from 3.6 to 4.2
    result.thin_max_width = static_cast<coord_t>(result.max_length_for_one_support_point * 2.5); 
    result.thick_min_width = static_cast<coord_t>(result.max_length_for_one_support_point * 2.15);

    // guessed from max_length_for_two_support_points to value 5.2mm
    result.thin_max_distance = static_cast<coord_t>(result.max_length_for_two_support_points * 0.8);

    // guess from experiments documented above __(not verified values)__
    result.thick_inner_max_distance = result.max_length_for_two_support_points; // 6.5mm
    result.thick_outline_max_distance = static_cast<coord_t>(result.max_length_for_two_support_points * 0.75); // 4.875mm

    result.minimal_distance_from_outline = result.head_radius;           // 0.2mm
    result.maximal_distance_from_outline = result.thin_max_distance / 3; // 1.73mm
    result.min_part_length = result.thin_max_distance;                   // 5.2mm

    // Align support points
    // TODO: propagate print resolution
    result.minimal_move = scale_(0.1); // 0.1 mm is enough
    // [in nanometers --> 0.01mm ], devide from print resolution to quater pixel is too strict
    result.count_iteration = 30; // speed VS precission
    result.max_align_distance = result.max_length_for_two_support_points / 2;

    // Defaults must be consistent on their own. Keep the call outside assert()
    // so a release build still runs it.
    [[maybe_unused]] const bool clean = verify(result);
    assert(clean);
    return result;
}

SampleConfig SampleConfigFactory::apply_min_distance(const SampleConfig &current, float min_distance_mm) {
    if (min_distance_mm <= 0.f)
        return current;

    const coord_t floor_distance = static_cast<coord_t>(scale_(min_distance_mm));
    SampleConfig result = current; // copy
    // These are MAXIMUM spacings - how far apart the sampler may leave two
    // points while still covering an island. Asking for a minimum only means
    // something if the maximum is at least as large, so each is raised to meet
    // the floor and none is ever lowered. An island too small to hold two
    // points that far apart still gets its one or two: those are decided by
    // max_length_for_*_support_point*, which are left alone.
    result.thin_max_distance = std::max(result.thin_max_distance, floor_distance);
    result.thick_inner_max_distance = std::max(result.thick_inner_max_distance, floor_distance);
    result.thick_outline_max_distance = std::max(result.thick_outline_max_distance, floor_distance);
    verify(result);
    return result;
}

SampleConfig SampleConfigFactory::apply_density(const SampleConfig &current, float density) {
    if (is_approx(density, 1.f))
        return current;
    if (density < .1f)
        density = .1f; // minimal 10%

    SampleConfig result = current;                                                        // copy
    result.thin_max_distance = static_cast<coord_t>(current.thin_max_distance / density); // linear
    result.thick_inner_max_distance = static_cast<coord_t>( // controll radius - quadratic
        std::sqrt(sqr((double) current.thick_inner_max_distance) / density)
    );
    result.thick_outline_max_distance = static_cast<coord_t>(
        current.thick_outline_max_distance / density
    ); // linear

    // verify() has no solution once thin_max_distance drops below
    // minimal_distance_from_outline (thin_max_width can't be both >= 2r+2m and
    // <= 2t+2r), so hold it there. minimal_move also floors heads under 0.2 mm,
    // where m is too small to keep the point count sane. The thick spacings
    // keep shrinking, floored so they never reach zero. The floors apply only
    // in this branch, so a density that keeps thin_max_distance >= m_thresh
    // gets exactly the three scaled values above.
    const coord_t m_thresh = std::max(current.minimal_distance_from_outline, current.minimal_move);
    if (result.thin_max_distance < m_thresh) {
        result.thin_max_distance = m_thresh;
        result.thick_inner_max_distance = std::max(result.thick_inner_max_distance,
            std::max(2 * current.head_radius, current.minimal_move));
        result.thick_outline_max_distance = std::max(result.thick_outline_max_distance,
            current.minimal_move);
    }
    // result.head_radius                       .. no change
    // result.minimal_distance_from_outline     .. no change
    // result.maximal_distance_from_outline     .. no change
    // result.max_length_for_one_support_point  .. no change
    // result.max_length_for_two_support_points .. no change
    verify(result);
    return result;
}

#ifdef USE_ISLAND_GUI_FOR_SETTINGS
std::optional<SampleConfig> SampleConfigFactory::gui_sample_config_opt;
SampleConfig &SampleConfigFactory::get_sample_config() {
    // init config
    if (!gui_sample_config_opt.has_value())
        // create default configuration
        gui_sample_config_opt = sla::SampleConfigFactory::create(.4f); 
    return *gui_sample_config_opt;
}

SampleConfig SampleConfigFactory::get_sample_config(float density) {
    return apply_density(get_sample_config(), density);
}
#endif // USE_ISLAND_GUI_FOR_SETTINGS