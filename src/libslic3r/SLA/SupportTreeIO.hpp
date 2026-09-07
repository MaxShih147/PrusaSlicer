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
// Shape (version 2):
//   {"version":2,"elevation":5.0,
//    "heads":[{"pos":[x,y,z],"dir":[x,y,z],"r_pin":0.2,"r_back":0.5,
//              "width":1.0,"penetration":0.2,"pillar":3}],
//    "pillars":[{"endpt":[x,y,z],"height":40.2,"r_start":0.5,"r_end":0.5,
//                "links":0,"bridges":0,"props":1,"head":4}],
//    "junctions":[{"pos":[x,y,z],"r":0.5,"pillar":3}],
//    "pedestals":[{"pos":[x,y,z],"height":1.0,"r_bottom":2.0,"r_top":0.5,
//                  "pillar":3}],
//    "bridges":[{"startp":[x,y,z],"endp":[x,y,z],"r":0.5,"end_r":0.5,
//                "reaches":7000,"a":1,"b":3}]}
//
// OWNERSHIP is by position in these lists: a head's "pillar" is an index into
// "pillars". The array subscript is the id, so nothing is spent carrying one.
// It is what makes an automatic run editable - hundreds of pins routed onto far
// fewer pillars, with no way to tell afterwards which pin a pillar carries
// unless the run says so.
//
// EVERY OWNERSHIP KEY IS OMITTED WHEN IT SAYS NOTHING (-1), and so is "end_r"
// when it equals "r" and "reaches" when it is -1. A reader fills those in with
// the documented defaults. On a plate of a few hundred supports this is most of
// the document: the common bar is a plain {"startp","endp","r"}. The dump is
// compact for the same reason - an automatic tree is downloaded whole, and
// indentation was a third of its bytes.
//
// `pillars` is index-aligned with the prior-pillar list reported alongside, so
// the caller's handle for pillars[i] is that list's [i].id. One numbering, not
// two.
//
// `reaches` on a bridge is the caller's handle for a pillar carried in from an
// earlier generation, or absent when both ends belong to this run. It is what
// lets the caller drop exactly the bars that die with a support it removes.

inline constexpr int support_tree_format_version = 2;

namespace detail {

inline nlohmann::json vec3(const Vec3d &v)
{
    return nlohmann::json::array({v.x(), v.y(), v.z()});
}

/// Write an ownership index only when there is one.
inline void put_owner(nlohmann::json &j, const char *key, int idx)
{
    if (idx >= 0)
        j[key] = idx;
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
        nlohmann::json j = {{"pos", vec3(h.pos)},
                            {"dir", vec3(h.dir)},
                            {"r_pin", h.r_pin},
                            {"r_back", h.r_back},
                            {"width", h.width},
                            {"penetration", h.penetration}};
        detail::put_owner(j, "pillar", h.pillar);
        heads.push_back(std::move(j));
    }

    nlohmann::json pillars = nlohmann::json::array();
    for (const TreePillar &p : els.pillars) {
        nlohmann::json j = {{"endpt", vec3(p.endpt)},
                            {"height", p.height},
                            {"r_start", p.r_start},
                            {"r_end", p.r_end},
                            {"links", p.links},
                            {"bridges", p.bridges}};
        detail::put_owner(j, "props", p.props);
        detail::put_owner(j, "head", p.head);
        pillars.push_back(std::move(j));
    }

    nlohmann::json junctions = nlohmann::json::array();
    for (const TreeJunction &j : els.junctions) {
        nlohmann::json rec = {{"pos", vec3(j.pos)}, {"r", j.r}};
        detail::put_owner(rec, "pillar", j.pillar);
        junctions.push_back(std::move(rec));
    }

    nlohmann::json pedestals = nlohmann::json::array();
    for (const TreePedestal &p : els.pedestals) {
        nlohmann::json j = {{"pos", vec3(p.pos)},
                            {"height", p.height},
                            {"r_bottom", p.r_bottom},
                            {"r_top", p.r_top}};
        detail::put_owner(j, "pillar", p.pillar);
        pedestals.push_back(std::move(j));
    }

    nlohmann::json bridges = nlohmann::json::array();
    for (const TreeBridge &b : els.bridges) {
        nlohmann::json j = {{"startp", vec3(b.startp)},
                            {"endp", vec3(b.endp)},
                            {"r", b.r}};
        // A bar tapers only when it joins two different radii; most do not, and
        // a reader takes end_r as r when it is absent.
        if (b.end_r != b.r)
            j["end_r"] = b.end_r;
        if (b.reaches >= 0)
            j["reaches"] = b.reaches;
        detail::put_owner(j, "a", b.a);
        detail::put_owner(j, "b", b.b);
        bridges.push_back(std::move(j));
    }

    nlohmann::json doc = {{"version", support_tree_format_version},
                          {"elevation", elevation},
                          {"heads", heads},
                          {"pillars", pillars},
                          {"junctions", junctions},
                          {"pedestals", pedestals},
                          {"bridges", bridges}};
    // Compact: an automatic tree is hundreds of elements downloaded whole, and
    // indentation was about a third of the bytes.
    return doc.dump();
}

}} // namespace Slic3r::sla

#endif // SLA_SUPPORTTREEIO_HPP
