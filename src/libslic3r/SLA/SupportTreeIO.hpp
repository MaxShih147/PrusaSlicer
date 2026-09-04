#ifndef SLA_SUPPORTTREEIO_HPP
#define SLA_SUPPORTTREEIO_HPP

#include <libslic3r/SLA/SupportTree.hpp>
#include <nlohmann/json.hpp>
#include <string>

namespace Slic3r { namespace sla {

// The support tree as data, for a caller that wants to draw and edit it rather
// than just print it.
//
// merged_mesh() is get_mesh() over these lists and nothing else, so this
// document says everything an STL of the same support says - and several things
// it cannot. An STL is one lump: there is no way to point at one bar of bracing
// in it, and no way to take that bar away. Here every element is its own record,
// so picking and deleting are a matter of an index.
//
// Positions are in the engine's own frame, the same one the exported STL uses:
// the mesh it was given, grounded, with the pillars landing one object elevation
// below the model's underside.
//
// Shape:
//   {"version":1,"elevation":5.0,
//    "heads":[{"pos":[x,y,z],"dir":[x,y,z],"r_pin":0.2,"r_back":0.5,
//              "width":1.0,"penetration":0.2}],
//    "pillars":[{"endpt":[x,y,z],"height":40.2,"r_start":0.5,"r_end":0.5,
//                "links":0,"bridges":0}],
//    "junctions":[{"pos":[x,y,z],"r":0.5}],
//    "pedestals":[{"pos":[x,y,z],"height":1.0,"r_bottom":2.0,"r_top":0.5}],
//    "bridges":[{"startp":[x,y,z],"endp":[x,y,z],"r":0.5,"end_r":0.5,
//                "reaches":7000}]}
//
// `pillars` is index-aligned with the prior-pillar list reported alongside, so
// the caller's handle for pillars[i] is that list's [i].id. One numbering, not
// two.
//
// `reaches` on a bridge is the caller's handle for a pillar carried in from an
// earlier generation, or -1 when both ends belong to this run. It is what lets
// the caller drop exactly the bars that die with a support it removes.

inline constexpr int support_tree_format_version = 1;

namespace detail {

inline nlohmann::json vec3(const Vec3d &v)
{
    return nlohmann::json::array({v.x(), v.y(), v.z()});
}

} // namespace detail

/**
 * Serialise a support tree's elements.
 * @param els what a generation produced
 * @param elevation how high the object sits above the plate for this config
 * @returns the JSON document
 */
inline std::string support_tree_to_string(const SupportTreeElements &els,
                                          double elevation = 0.)
{
    using detail::vec3;

    nlohmann::json heads = nlohmann::json::array();
    for (const TreeHead &h : els.heads) {
        heads.push_back({{"pos", vec3(h.pos)},
                         {"dir", vec3(h.dir)},
                         {"r_pin", h.r_pin},
                         {"r_back", h.r_back},
                         {"width", h.width},
                         {"penetration", h.penetration}});
    }

    nlohmann::json pillars = nlohmann::json::array();
    for (const TreePillar &p : els.pillars) {
        pillars.push_back({{"endpt", vec3(p.endpt)},
                           {"height", p.height},
                           {"r_start", p.r_start},
                           {"r_end", p.r_end},
                           {"links", p.links},
                           {"bridges", p.bridges}});
    }

    nlohmann::json junctions = nlohmann::json::array();
    for (const TreeJunction &j : els.junctions)
        junctions.push_back({{"pos", vec3(j.pos)}, {"r", j.r}});

    nlohmann::json pedestals = nlohmann::json::array();
    for (const TreePedestal &p : els.pedestals) {
        pedestals.push_back({{"pos", vec3(p.pos)},
                             {"height", p.height},
                             {"r_bottom", p.r_bottom},
                             {"r_top", p.r_top}});
    }

    nlohmann::json bridges = nlohmann::json::array();
    for (const TreeBridge &b : els.bridges) {
        bridges.push_back({{"startp", vec3(b.startp)},
                           {"endp", vec3(b.endp)},
                           {"r", b.r},
                           {"end_r", b.end_r},
                           {"reaches", b.reaches}});
    }

    nlohmann::json doc = {{"version", support_tree_format_version},
                          {"elevation", elevation},
                          {"heads", heads},
                          {"pillars", pillars},
                          {"junctions", junctions},
                          {"pedestals", pedestals},
                          {"bridges", bridges}};
    return doc.dump(1, ' ');
}

}} // namespace Slic3r::sla

#endif // SLA_SUPPORTTREEIO_HPP
