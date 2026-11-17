#ifndef SLA_PRIORPILLARIO_HPP
#define SLA_PRIORPILLARIO_HPP

#include <libslic3r/SLA/SupportTree.hpp>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cstdint>
#include <limits>
#include <string>

namespace Slic3r { namespace sla {

// Reading the pillars of an already generated support.
//
// Deliberately separate from SupportPointIO: a support POINT is what the user
// placed, a prior PILLAR is what the engine grew from it. They travel together
// but they are not the same thing, and only the pillar carries what bracing
// needs (where it stands, how thick it is, how many links it already has).
//
// Shape (version 1):
//   {"version":1,"elevation":5.0,
//    "attached_to":[{"prior_id":7,"links":1,"bridges":0,"pairs":2}],
//    "pillars":[
//     {"id":7,"endpoint":[x,y,z],"height":40.2,"r_start":0.5,
//      "r_end":0.5,"links":0,"bridges":0,"pairs":0}]}
//
// "elevation" and "attached_to" are written, never read.
//
// `pairs` counts the pillars one is braced to, a much shorter neighbour
// included; it is what caps further bracing, while `links` stays the
// structural count. It came later than the rest, so a list without it is
// still read: its pairs are taken as its links. Every link is a pair, so a
// list that says fewer pairs than links is lifted to links.
//
// `id` is the caller's handle, echoed nowhere by the engine and never
// interpreted by it - it exists so the caller can be told what a new support
// attached to.

inline constexpr int prior_pillar_format_version = 1;

/**
 * Parse a prior pillar list.
 * @param text JSON document
 * @param out parsed pillars, untouched on failure
 * @param err human readable reason, cleared on success
 * @returns whether the document was accepted
 */
namespace detail {

// A pillar's pairs, or `links` when there is none to take. The key came after
// links, so a list from before it has none; and a caller's null (what
// JSON.stringify makes of NaN), negative number, fraction or string is taken
// as none too, rather than throwing out of the parser or wrapping round to a
// pillar that counts as braced to four billion others.
inline unsigned read_pairs(const nlohmann::json &pj, unsigned links)
{
    const auto it = pj.find("pairs");
    if (it == pj.end() || !it->is_number_unsigned()) return links;
    const auto v = it->template get<std::uint64_t>();
    return v > std::numeric_limits<unsigned>::max() ? links : unsigned(v);
}

} // namespace detail

inline bool prior_pillars_from_string(const std::string &text,
                                      PriorPillars &out,
                                      std::string &err)
{
    err.clear();

    nlohmann::json j = nlohmann::json::parse(text, nullptr, false);
    if (j.is_discarded() || !j.is_object()) {
        err = "prior pillar file is not a JSON object";
        return false;
    }

    const auto vit = j.find("version");
    if (vit == j.end() || !vit->is_number_integer()
        || vit->template get<int64_t>() != prior_pillar_format_version) {
        err = "prior pillar file has a missing or unsupported version";
        return false;
    }

    const auto pit = j.find("pillars");
    if (pit == j.end() || !pit->is_array()) {
        err = "prior pillar file has no pillars array";
        return false;
    }

    PriorPillars parsed;
    parsed.reserve(pit->size());
    for (const nlohmann::json &pj : *pit) {
        if (!pj.is_object()) {
            err = "a pillar entry is not an object";
            return false;
        }
        const auto eit = pj.find("endpoint");
        if (eit == pj.end() || !eit->is_array() || eit->size() != 3) {
            err = "a pillar entry has no 3-element endpoint";
            return false;
        }

        PriorPillar p;
        p.endpoint = Vec3d((*eit)[0].template get<double>(),
                           (*eit)[1].template get<double>(),
                           (*eit)[2].template get<double>());
        p.id      = pj.value("id", -1);
        p.height  = pj.value("height", 0.);
        p.r_start = pj.value("r_start", 0.);
        // A pillar of zero radius would brace to nothing and divide by nothing;
        // refusing beats generating something the caller cannot explain.
        if (p.height <= 0. || p.r_start <= 0.) {
            err = "a pillar entry has a non-positive height or r_start";
            return false;
        }
        p.r_end   = pj.value("r_end", p.r_start);
        p.links   = unsigned(pj.value("links", 0));
        p.bridges = unsigned(pj.value("bridges", 0));
        p.pairs   = std::max(detail::read_pairs(pj, p.links), p.links);
        parsed.push_back(p);
    }

    out = std::move(parsed);
    return true;
}

/**
 * Serialise pillars so a later generation can be handed them as prior.
 * @param pillars what a generation produced
 * @returns the JSON document
 */
inline std::string prior_pillars_to_string(const PriorPillars &pillars,
                                           double elevation = 0.,
                                           const PriorAttachments &attached = {})
{
    nlohmann::json arr = nlohmann::json::array();
    for (const PriorPillar &p : pillars) {
        arr.push_back({
            {"id", p.id},
            {"endpoint", {p.endpoint.x(), p.endpoint.y(), p.endpoint.z()}},
            {"height", p.height},
            {"r_start", p.r_start},
            {"r_end", p.r_end},
            {"links", p.links},
            {"bridges", p.bridges},
            {"pairs", p.pairs},
        });
    }
    // How high the object sits above the plate for this configuration. It is a
    // property of the settings, not of the support that grew - the caller needs
    // it BEFORE any support exists, to place the model. Comes straight from
    // SLAPrintObject::get_elevation() so nobody has to reproduce the rule that
    // an enabled pad adds its wall thickness on top of the object elevation.
    // What this run braced to. The link counts are the ones those pillars carry
    // NOW, pair counts too, so the caller can update its own copies before the
    // next generation - stale counts make the engine keep bracing to a pillar
    // that is full.
    nlohmann::json att = nlohmann::json::array();
    for (const PriorAttachment &a : attached) {
        att.push_back({{"prior_id", a.prior_id},
                       {"links", a.links},
                       {"bridges", a.bridges},
                       {"pairs", a.pairs}});
    }

    nlohmann::json doc = {{"version", prior_pillar_format_version},
                          {"elevation", elevation},
                          {"attached_to", att},
                          {"pillars", arr}};
    return doc.dump(1, ' ');
}

}} // namespace Slic3r::sla

#endif // SLA_PRIORPILLARIO_HPP
