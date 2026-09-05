///|/ Copyright (c) Prusa Research 2020 - 2023 Oleksandra Iushchenko @YuSanka, Tomáš Mészáros @tamasmeszaros, Roman Beránek @zavorka, Vojtěch Bubník @bubnikv
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
/**
 * In this file we will implement the automatic SLA support tree generation.
 *
 */

#include <unordered_map>
#include <libslic3r/SLA/SupportTree.hpp>
#include <libslic3r/SLA/SupportTreeBuilder.hpp>
#include <libslic3r/SLA/DefaultSupportTree.hpp>
#include <libslic3r/SLA/BranchingTreeSLA.hpp>
#include <libslic3r/MTUtils.hpp>
#include <libslic3r/TriangleMeshSlicer.hpp>
#include <boost/log/trivial.hpp>
#include <chrono>
#include <iterator>
#include <cstddef>

#include "libslic3r/Point.hpp"
#include "libslic3r/SLA/JobController.hpp"
#include "libslic3r/SLA/Pad.hpp"
#include "libslic3r/SLA/SupportTreeStrategies.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/libslic3r.h"


namespace Slic3r { namespace sla {

/// Which caller handle did a frozen builder pillar come in under?
///
/// adopt_prior_pillars() registers SupportableMesh::prior in order, so the nth
/// frozen pillar in the builder is the nth entry of prior.
static long prior_caller_id(const SupportableMesh &sm,
                            const SupportTreeBuilder &builder, long pillar_id)
{
    size_t seen = 0;
    const auto &pillars = builder.pillars();
    for (size_t i = 0; i < pillars.size(); ++i) {
        if (!pillars[i].frozen) continue;
        if (long(i) == pillar_id)
            return seen < sm.prior.size() ? sm.prior[seen].id : -1;
        ++seen;
    }
    return -1;
}

indexed_triangle_set create_support_tree(const SupportableMesh &sm,
                                         const JobController   &ctl,
                                         PriorPillars          *out_pillars,
                                         PriorAttachments      *out_attached,
                                         FrozenBraceMeshes     *out_braces,
                                         SupportTreeElements   *out_elements,
                                         indexed_triangle_set  *out_full_mesh)
{
    auto builder = make_unique<SupportTreeBuilder>(ctl);

    if (sm.cfg.enabled) {
        using std::chrono::high_resolution_clock;
        auto start{high_resolution_clock::now()};

        switch (sm.cfg.tree_type) {
        case SupportTreeType::Default: {
            DefaultSupportTree::execute(*builder, sm, out_attached);
            break;
        }
        case SupportTreeType::Branching: {
            create_branching_tree(*builder, sm);
            break;
        }
        case SupportTreeType::Organic: {
            // TODO
        }
        default:;
        }

        auto stop{high_resolution_clock::now()};

        using std::chrono::duration;
        using std::chrono::seconds;
        BOOST_LOG_TRIVIAL(info) << "Support tree creation took: "
                                << duration<double>{stop - start}.count()
                                << " seconds";

        // Capture the pillars BEFORE cleanup, which frees the logical tree.
        // Frozen pillars are skipped: they came from the caller and echoing
        // them back would double-count them on the next generation.
        if (out_pillars) {
            out_pillars->clear();
            long next_id = 0;
            for (const Pillar &p : builder->pillars()) {
                if (p.frozen) continue;
                PriorPillar pp;
                pp.id       = next_id++;
                pp.endpoint = p.endpoint();
                pp.height   = p.height;
                pp.r_start  = p.r_start;
                pp.r_end    = p.r_end;
                pp.links    = p.links;
                pp.bridges  = p.bridges;
                out_pillars->push_back(pp);
            }
        }

        // Also before cleanup: the braces reaching frozen pillars, keyed by the
        // caller's handle for each. Mapped here because only the algorithm knows
        // which builder pillar came in under which handle.
        if (out_braces) {
            out_braces->clear();
            const auto by_pillar = builder->frozen_brace_meshes();
            for (const auto &[pid, mesh] : by_pillar) {
                const long caller = prior_caller_id(sm, *builder, pid);
                if (caller >= 0)
                    (*out_braces)[caller] = mesh;
            }
        }

        // The tree as data, captured before cleanup like everything else here.
        // This is the same content merged_mesh() is about to turn into
        // triangles - handing it over lets the caller draw the support itself,
        // point at one bar of bracing, and remove it without another run.
        if (out_elements) {
            *out_elements = SupportTreeElements{};

            // Ownership travels as a POSITION in the exported list, not as the
            // builder's own id: invalid heads and frozen pillars are left out,
            // so the two numberings differ. This map is what keeps every
            // reference below pointing at the right record.
            std::unordered_map<long, int> pillar_slot;
            std::unordered_map<long, int> head_slot;
            {
                int i = 0;
                for (const Head &h : builder->heads())
                    if (h.is_valid()) head_slot[h.id] = i++;
            }
            auto head_at = [&head_slot](long id) {
                auto it = head_slot.find(id);
                return it == head_slot.end() ? -1 : it->second;
            };

            // Frozen pillars are the caller's own from earlier generations;
            // echoing them would have it draw each one twice. Skipping them here
            // keeps this list index-aligned with out_pillars, which skips them
            // for the same reason.
            for (const Pillar &p : builder->pillars()) {
                if (p.frozen) continue;
                pillar_slot[p.id] = int(out_elements->pillars.size());
                out_elements->pillars.push_back(
                    TreePillar{p.endpoint(), p.height, p.r_start, p.r_end,
                               p.links, p.bridges, -1,
                               p.starts_from_head ? head_at(p.start_junction_id) : -1});
            }

            auto slot = [&pillar_slot](long id) {
                auto it = pillar_slot.find(id);
                return it == pillar_slot.end() ? -1 : it->second;
            };

            // Second pass, now that every pillar has a slot: an auxiliary pillar
            // says which one it props up.
            {
                int i = 0;
                for (const Pillar &p : builder->pillars()) {
                    if (p.frozen) continue;
                    out_elements->pillars[size_t(i++)].props = slot(p.props_for);
                }
            }

            for (const Head &h : builder->heads()) {
                if (!h.is_valid()) continue;
                // Where the load goes: its own pillar, or the one it bridges to.
                long carrier = h.pillar_id;
                if (carrier < 0 && h.bridge_id >= 0
                    && size_t(h.bridge_id) < builder->bridges().size()) {
                    carrier = builder->bridges()[size_t(h.bridge_id)].owner_b;
                }
                out_elements->heads.push_back(
                    TreeHead{h.pos, h.dir, h.r_pin_mm, h.r_back_mm, h.width_mm,
                             h.penetration_mm, slot(carrier)});
            }

            for (const Junction &j : builder->junctions())
                out_elements->junctions.push_back(
                    TreeJunction{j.pos, j.r, slot(j.pillar_id)});

            for (const Pedestal &p : builder->pedestals())
                out_elements->pedestals.push_back(
                    TreePedestal{p.pos, p.height, p.r_bottom, p.r_top,
                                 slot(p.pillar_id)});

            // Bracing, one record per BAR. A bar reaching a pillar the caller
            // carried in is tagged with that pillar's handle: it belongs to both
            // ends, and the caller has to be able to drop it when that end goes.
            auto add_bridge = [&](const Bridge &b, double end_r) {
                long reaches = -1;
                if (builder->bridge_reaches_frozen(b))
                    reaches = prior_caller_id(sm, *builder,
                                              builder->bridge_frozen_target(b));
                out_elements->bridges.push_back(
                    TreeBridge{b.startp, b.endp, b.r, end_r, reaches,
                               slot(b.owner_a), slot(b.owner_b)});
            };

            for (const Bridge &b : builder->bridges())      add_bridge(b, b.r);
            for (const Bridge &b : builder->crossbridges()) add_bridge(b, b.r);
            for (const DiffBridge &b : builder->diffbridges()) add_bridge(b, b.end_r);
        }

        // The whole tree, frozen pillars included, for the pad to be grown from.
        // Also before cleanup, for the same reason as everything else here.
        if (out_full_mesh)
            *out_full_mesh = builder->full_mesh();

        builder->merge_and_cleanup();   // clean metadata, leave only the meshes.
    }

    indexed_triangle_set out = builder->retrieve_mesh(MeshType::Support);

    return out;
}

indexed_triangle_set create_pad(const SupportableMesh      &sm,
                                const indexed_triangle_set &support_mesh,
                                const JobController        &ctl)
{
    constexpr float PadSamplingLH = 0.1f;

    ExPolygons model_contours; // This will store the base plate of the pad.
    double pad_h  = sm.pad_cfg.full_height();
    auto   gndlvl = float(ground_level(sm));
    float  zstart = gndlvl - bool(sm.pad_cfg.embed_object) * sm.pad_cfg.wall_thickness_mm;
    float  zend   = zstart + float(pad_h + PadSamplingLH + EPSILON);
    auto  heights = grid(zstart, zend, PadSamplingLH);

    if (!sm.cfg.enabled || sm.pad_cfg.embed_object) {
        // No support (thus no elevation) or zero elevation mode
        // we sometimes call it "builtin pad" is enabled so we will
        // get a sample from the bottom of the mesh and use it for pad
        // creation.
        sla::pad_blueprint(*sm.emesh.get_triangle_mesh(), model_contours,
                           heights, ctl.cancelfn);
    }

    ExPolygons sup_contours;
    pad_blueprint(support_mesh, sup_contours, heights, ctl.cancelfn);

    indexed_triangle_set out;
    create_pad(sup_contours, model_contours, out, sm.pad_cfg);

    Vec3f offs{.0f, .0f, gndlvl};
    for (auto &p : out.vertices) p += offs;

    its_merge_vertices(out);

    return out;
}

std::vector<ExPolygons> slice(const indexed_triangle_set &sup_mesh,
                              const indexed_triangle_set &pad_mesh,
                              const std::vector<float>   &grid,
                              float                       cr,
                              const JobController        &ctl)
{
    using Slices = std::vector<ExPolygons>;

    auto slices = reserve_vector<Slices>(2);

    if (!sup_mesh.empty()) {
        slices.emplace_back();
        slices.back() = slice_mesh_ex(sup_mesh, grid, cr, ctl.cancelfn);
    }

    if (!pad_mesh.empty()) {
        slices.emplace_back();

        auto bb     = bounding_box(pad_mesh);
        auto maxzit = std::upper_bound(grid.begin(), grid.end(), bb.max.z());

        auto cap     = grid.end() - maxzit;
        auto padgrid = reserve_vector<float>(size_t(cap > 0 ? cap : 0));
        std::copy(grid.begin(), maxzit, std::back_inserter(padgrid));

        slices.back() = slice_mesh_ex(pad_mesh, padgrid, cr, ctl.cancelfn);
    }

    size_t len = grid.size();
    for (const Slices &slv : slices)
        len = std::min(len, slv.size());

    // Either the support or the pad or both has to be non empty
    if (slices.empty()) return {};

    Slices &mrg = slices.front();

    for (auto it = std::next(slices.begin()); it != slices.end(); ++it) {
        for (size_t i = 0; i < len; ++i) {
            Slices &slv = *it;
            std::copy(slv[i].begin(), slv[i].end(), std::back_inserter(mrg[i]));
            slv[i] = {}; // clear and delete
        }
    }

    return mrg;
}

}} // namespace Slic3r::sla
