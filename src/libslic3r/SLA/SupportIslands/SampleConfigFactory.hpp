#ifndef slic3r_SLA_SuppotstIslands_SampleConfigFactory_hpp_
#define slic3r_SLA_SuppotstIslands_SampleConfigFactory_hpp_

#include <optional>
#include "SampleConfig.hpp"
#include "libslic3r/PrintConfig.hpp"

//#define USE_ISLAND_GUI_FOR_SETTINGS

namespace Slic3r::sla {

/// <summary>
/// Factory to create configuration
/// </summary>
class SampleConfigFactory
{
public:
    SampleConfigFactory() = delete;

    static bool verify(SampleConfig &cfg);
    static SampleConfig create(float support_head_diameter_in_mm);
    static SampleConfig apply_density(const SampleConfig& cfg, float density);

    /// <summary>
    /// Raise the sampler's spacings so they never sit below a floor the user
    /// asked for.
    /// </summary>
    /// <param name="cfg">Configuration to raise</param>
    /// <param name="min_distance_mm">The floor; 0 leaves the config alone</param>
    /// <returns>A copy with no spacing below the floor</returns>
    static SampleConfig apply_min_distance(const SampleConfig& cfg, float min_distance_mm);
#ifdef USE_ISLAND_GUI_FOR_SETTINGS
private:
    // TODO: REMOVE IT. Do not use in production
    // Global variable to temporary set configuration from GUI into SLA print steps
    static std::optional<SampleConfig> gui_sample_config_opt;
public:
    static SampleConfig &get_sample_config();

    /// <summary>
    /// Create scaled copy of sample config
    /// </summary>
    /// <param name="density"> Scale for config values(minimal value is .1f)
    /// 1.f .. no scale
    /// .9f .. less support points (approx 90%)
    /// 1.1f.. extend count of supports (approx to 110%) </param>
    /// <returns>Scaled configuration</returns>
    static SampleConfig get_sample_config(float density);
#endif // USE_ISLAND_GUI_FOR_SETTINGS
};
} // namespace Slic3r::sla
#endif // slic3r_SLA_SuppotstIslands_SampleConfigFactory_hpp_
