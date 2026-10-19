#include "BracingPlanner.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>

#include <libslic3r/BoundingBox.hpp>
#include <libslic3r/libslic3r.h>

namespace Slic3r { namespace sla { namespace bracing {

const char *reason_name(Reason r)
{
    switch (r) {
    case Reason::accepted:        return "accepted";
    case Reason::too_close:       return "too_close";
    case Reason::pair_cap:        return "pair_cap";
    case Reason::obstacle_pillar: return "obstacle_pillar";
    case Reason::shared_angle:    return "shared_angle";
    case Reason::cross_truss:     return "cross_truss";
    case Reason::z_clear_fail:    return "z_clear_fail";
    case Reason::mesh_hit:        return "mesh_hit";
    case Reason::no_room:         return "no_room";
    case Reason::out_of_range:    return "out_of_range";
    }
    return "?";
}

// ─── geometry ────────────────────────────────────────────────────────────────

namespace geom {

namespace {

double cross2(const Vec2d &p, const Vec2d &q) { return p.x() * q.y() - p.y() * q.x(); }

// Parameter of the point of [a,b] closest to p; a segment shorter than
// EPS_LEN is the point a.
double closest_param(const Vec2d &p, const Vec2d &a, const Vec2d &b)
{
    const Vec2d  u  = b - a;
    const double l2 = u.squaredNorm();
    if (l2 < EPS_LEN) return 0.;
    return std::clamp((p - a).dot(u) / l2, 0., 1.);
}

} // namespace

double point_seg_dist2d(const Vec2d &p, const Vec2d &a, const Vec2d &b)
{
    const Vec2d u = b - a;
    if (u.squaredNorm() < EPS_LEN) return (p - a).norm();
    const double t = closest_param(p, a, b);
    return (p - (a + t * u)).norm();
}

double seg_seg_closest2d(const Vec2d &a, const Vec2d &b, const Vec2d &c,
                         const Vec2d &d, double &s, double &t)
{
    const Vec2d  u = b - a, v = d - c, w = c - a;
    const double den = cross2(u, v);

    if (std::abs(den) > EPS_PAR * u.norm() * v.norm()) {
        const double ss = cross2(w, v) / den;
        const double tt = cross2(w, u) / den;
        if (ss >= 0. && ss <= 1. && tt >= 0. && tt <= 1.) {
            s = ss; t = tt;
            return 0.; // a real crossing, a T or two touching ends
        }
    }

    // Segments that do not meet are closest at an end of one of them; this
    // also covers parallel, collinear and zero-length segments.
    const double ta = closest_param(a, c, d), tb = closest_param(b, c, d);
    const double sc = closest_param(c, a, b), sd = closest_param(d, a, b);
    const double da = point_seg_dist2d(a, c, d), db = point_seg_dist2d(b, c, d);
    const double dc = point_seg_dist2d(c, a, b), dd = point_seg_dist2d(d, a, b);

    double best = da; s = 0.; t = ta;
    if (db < best) { best = db; s = 1.; t = tb; }
    if (dc < best) { best = dc; s = sc; t = 0.; }
    if (dd < best) { best = dd; s = sd; t = 1.; }
    return best;
}

double seg_seg_dist2d(const Vec2d &a, const Vec2d &b, const Vec2d &c, const Vec2d &d)
{
    double s, t;
    return seg_seg_closest2d(a, b, c, d, s, t);
}

double seg_seg_dist3d(const Vec3d &p1, const Vec3d &q1, const Vec3d &p2, const Vec3d &q2)
{
    double s, t;
    return seg_seg_closest3d(p1, q1, p2, q2, s, t);
}

double seg_seg_closest3d(const Vec3d &p1, const Vec3d &q1, const Vec3d &p2,
                         const Vec3d &q2, double &s, double &t)
{
    const Vec3d  d1 = q1 - p1, d2 = q2 - p2, r = p1 - p2;
    const double a = d1.dot(d1), e = d2.dot(d2), f = d2.dot(r);

    if (a <= EPS_LEN && e <= EPS_LEN) {
        s = t = 0.;
    } else if (a <= EPS_LEN) {
        s = 0.;
        t = std::clamp(f / e, 0., 1.);
    } else {
        const double c = d1.dot(r);
        if (e <= EPS_LEN) {
            t = 0.;
            s = std::clamp(-c / a, 0., 1.);
        } else {
            const double b = d1.dot(d2), den = a * e - b * b;
            // Parallel: any s will do, the clamping of t below fixes it up.
            s = den > EPS_PAR * a * e ? std::clamp((b * f - c * e) / den, 0., 1.) : 0.;
            t = (b * s + f) / e;
            if (t < 0.) {
                t = 0.;
                s = std::clamp(-c / a, 0., 1.);
            } else if (t > 1.) {
                t = 1.;
                s = std::clamp((b - c) / a, 0., 1.);
            }
        }
    }

    return ((p1 + s * d1) - (p2 + t * d2)).norm();
}

double plan_closest_dz(const Segment3 &bar1, const Segment3 &bar2)
{
    double s, t;
    seg_seg_closest2d(bar1.a.head<2>(), bar1.b.head<2>(), bar2.a.head<2>(),
                      bar2.b.head<2>(), s, t);
    const double z1 = bar1.a.z() + s * (bar1.b.z() - bar1.a.z());
    const double z2 = bar2.a.z() + t * (bar2.b.z() - bar2.a.z());
    return z1 - z2;
}

bool shared_angle_ok(const Vec2d &shared, const Vec2d &end1, const Vec2d &end2,
                     double cos_min)
{
    return dir_angle_ok(end1 - shared, end2 - shared, cos_min);
}

bool dir_angle_ok(const Vec2d &u, const Vec2d &v, double cos_min)
{
    const double lu = u.squaredNorm(), lv = v.squaredNorm();
    // No direction to measure from: not a pair that can be judged as fine.
    if (lu < EPS_LEN || lv < EPS_LEN) return false;
    const double c = std::clamp(u.dot(v) / (std::sqrt(lu) * std::sqrt(lv)), -1., 1.);
    return c <= cos_min; // exactly the limit angle passes
}

std::vector<Segment3> truss_bars(const PillarGeom &p, const PillarGeom &q,
                                 const TrussParams &prm)
{
    // Line by line the layout of DefaultSupportTree::interconnect(), with the
    // model check left to the caller and the radius the thinner of the two.
    std::vector<Segment3> out;
    const double r = std::min(p.r, q.r);

    Vec3d supper{p.xy.x(), p.xy.y(), p.top_z()};
    Vec3d slower{q.xy.x(), q.xy.y(), q.top_z()};
    Vec3d eupper{p.xy.x(), p.xy.y(), p.foot_z};
    Vec3d elower{q.xy.x(), q.xy.y(), q.foot_z};

    const double zmin = prm.ground_z + prm.base_height;
    eupper.z() = std::max(eupper.z(), zmin);
    elower.z() = std::max(elower.z(), zmin);

    // The usable length of both pillars should be positive
    if (slower.z() - elower.z() < 0) return out;
    if (supper.z() - eupper.z() < 0) return out;

    const double pillar_dist = (Vec2d{slower.x(), slower.y()} -
                                Vec2d{supper.x(), supper.y()}).norm();
    const double zstep = pillar_dist * std::tan(-prm.bridge_slope_rad);

    // A zero distance or a flat slope would never step down: no bars, rather
    // than a loop that does not end.
    if (!(zstep < 0.)) return out;

    if (supper.z() < slower.z()) std::swap(supper, slower);
    if (eupper.z() < elower.z()) std::swap(eupper, elower);

    double startz = slower.z() - zstep < supper.z() ? slower.z() - zstep : slower.z();

    if (slower.z() - eupper.z() < std::abs(zstep)) {
        // no space for even one cross: get the most room and centre on it
        startz = std::min(supper.z(), slower.z() - zstep);
        const double endz = std::max(eupper.z() + zstep, elower.z());
        const double available_dist = startz - endz;
        const double rounds = std::floor(available_dist / std::abs(zstep));
        startz -= 0.5 * (available_dist - rounds * std::abs(zstep));
    }

    const bool docrosses =
        prm.mode == PillarConnectionMode::cross ||
        (prm.mode == PillarConnectionMode::dynamic && pillar_dist > 2 * prm.base_radius);

    Vec3d sj = supper, ej = slower;
    sj.z() = startz;
    ej.z() = sj.z() + zstep;

    while (ej.z() >= eupper.z()) {
        out.push_back({sj, ej, r});

        if (docrosses) {
            const Vec3d sjback(ej.x(), ej.y(), sj.z());
            const Vec3d ejback(sj.x(), sj.y(), ej.z());
            if (sjback.z() <= slower.z() && ejback.z() >= eupper.z())
                out.push_back({sjback, ejback, r});
        }

        std::swap(sj, ej);
        ej.z() = sj.z() + zstep;
    }

    return out;
}

} // namespace geom

// ─── planner ─────────────────────────────────────────────────────────────────

namespace {

// Plan-view box of [a,b] grown by pad, rounded outwards so that nothing on
// the edge is lost to the integer grid.
BoundingBox plan_box(const Vec2d &a, const Vec2d &b, double pad)
{
    const double x0 = std::min(a.x(), b.x()) - pad, x1 = std::max(a.x(), b.x()) + pad;
    const double y0 = std::min(a.y(), b.y()) - pad, y1 = std::max(a.y(), b.y()) + pad;
    return BoundingBox{Point{coord_t(std::floor(scale_(x0))), coord_t(std::floor(scale_(y0)))},
                       Point{coord_t(std::ceil(scale_(x1))), coord_t(std::ceil(scale_(y1)))}};
}

Vec2d plan(const Vec3d &v) { return v.head<2>(); }

bool contains(const std::vector<size_t> &roots, long root)
{
    return root >= 0 && std::find(roots.begin(), roots.end(), size_t(root)) != roots.end();
}

} // namespace

BracingPlanner::BracingPlanner(PlannerConfig cfg, MeshClearFn mesh_clear)
    : m_cfg{std::move(cfg)}, m_mesh_clear{std::move(mesh_clear)}
{}

void BracingPlanner::add_pillar(const PillarGeom &p)
{
    assert(!has_pillar(p.id));
    const size_t idx = m_pillars.size();
    m_pillars.emplace_back(p);
    m_pillar_at[p.id] = idx;
    m_parent.push_back(idx);
    m_members[idx] = {idx};

    // The boxes in the index carry each pillar's own radius, so growing the
    // query by p's radius and the clearance finds every pillar it can hug.
    const double c = m_cfg.safety_clearance;
    for (const BoxIndexEl &el : m_pillar_idx.query(plan_box(p.xy, p.xy, p.r + c),
                                                   BoxIndex::qtIntersects)) {
        const PillarGeom &q = m_pillars[el.second];
        if ((p.xy - q.xy).norm() < p.r + q.r + c) unite(idx, el.second);
    }

    m_pillar_idx.insert(plan_box(p.xy, p.xy, p.r), unsigned(idx));
}

size_t BracingPlanner::find_root(size_t idx) const
{
    while (m_parent[idx] != idx) idx = m_parent[idx];
    return idx;
}

void BracingPlanner::unite(size_t i, size_t j)
{
    size_t ri = find_root(i), rj = find_root(j);
    if (ri == rj) return;
    // The lower index stays the root: the same input, the same roots.
    if (rj < ri) std::swap(ri, rj);
    m_parent[rj] = ri;
    std::vector<size_t> &into = m_members[ri];
    const std::vector<size_t> &from = m_members[rj];
    into.insert(into.end(), from.begin(), from.end());
    m_members.erase(rj);
}

long BracingPlanner::cluster_of(long id) const { return root_of_id(id); }

long BracingPlanner::root_of_id(long id) const
{
    const auto it = m_pillar_at.find(id);
    return it == m_pillar_at.end() ? -1 : long(find_root(it->second));
}

std::vector<size_t> BracingPlanner::roots_of(const PillarGeom &p) const
{
    if (has_pillar(p.id)) return {find_root(m_pillar_at.at(p.id))};

    std::vector<size_t> out;
    const double c = m_cfg.safety_clearance;
    for (const BoxIndexEl &el : m_pillar_idx.query(plan_box(p.xy, p.xy, p.r + c),
                                                   BoxIndex::qtIntersects)) {
        const PillarGeom &q = m_pillars[el.second];
        if ((p.xy - q.xy).norm() < p.r + q.r + c) out.push_back(find_root(el.second));
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

void BracingPlanner::add_obstacle_bar(const ObstacleBar &b)
{
    const size_t idx = m_obstacles.size();
    m_obstacles.emplace_back(b);
    m_bar_idx.insert(plan_box(plan(b.seg.a), plan(b.seg.b), b.seg.r), unsigned(idx));
}

void BracingPlanner::add_truss_bar(long a, long b, const Segment3 &bar)
{
    // A bar that cannot be tied to two known pillars cannot be judged as a
    // group; it still takes part as an ordinary bar.
    if (a == b || !has_pillar(a) || !has_pillar(b)) {
        add_obstacle_bar({bar, a, b});
        return;
    }

    const long lo = std::min(a, b), hi = std::max(a, b);
    int found = -1;
    for (int ti : m_truss_of[lo])
        if (m_built[size_t(ti)].a == lo && m_built[size_t(ti)].b == hi) found = ti;

    if (found < 0) {
        BuiltTruss t;
        t.a = lo; t.b = hi; t.r = bar.r; t.existing = true;
        found = int(m_built.size());
        m_built.emplace_back(std::move(t));
        m_truss_of[lo].push_back(found);
        m_truss_of[hi].push_back(found);
        m_truss_idx.insert(plan_box(pillar(lo).xy, pillar(hi).xy, bar.r), unsigned(found));
    } else if (bar.r > m_built[size_t(found)].r) {
        // The box in the index carries the group's radius; widen it.
        BuiltTruss &t = m_built[size_t(found)];
        m_truss_idx.remove({plan_box(pillar(lo).xy, pillar(hi).xy, t.r), unsigned(found)});
        t.r = bar.r;
        m_truss_idx.insert(plan_box(pillar(lo).xy, pillar(hi).xy, t.r), unsigned(found));
    }

    m_built[size_t(found)].bars.push_back(bar);
}

const PillarGeom &BracingPlanner::pillar(long id) const { return m_pillars[m_pillar_at.at(id)]; }
PillarGeom &BracingPlanner::pillar_mut(long id) { return m_pillars[m_pillar_at.at(id)]; }

bool BracingPlanner::built_pair(long a, long b) const
{
    const long lo = std::min(a, b), hi = std::max(a, b);
    const auto it = m_truss_of.find(lo);
    if (it == m_truss_of.end()) return false;
    for (int ti : it->second)
        if (m_built[size_t(ti)].a == lo && m_built[size_t(ti)].b == hi) return true;
    return false;
}

bool BracingPlanner::rescue_target(const PillarGeom &p) const
{
    return p.braceable && !p.frozen && p.links == 0 && p.height > m_cfg.rescue_height;
}

// The structural count's rule: a much shorter neighbour does not hold a tall
// pillar up.
bool BracingPlanner::gives_link(const PillarGeom &p, const PillarGeom &other) const
{
    return p.height < m_cfg.solo_height || other.height / p.height > m_cfg.min_height_ratio;
}

std::vector<CandidateEdge> BracingPlanner::collect(const std::vector<long> &from) const
{
    std::vector<CandidateEdge> out;
    const double reach = m_cfg.max_link_distance;

    for (long id : from) {
        if (!has_pillar(id)) continue;
        const PillarGeom &p = pillar(id);
        if (p.frozen || !p.braceable) continue; // a frozen pillar never asks

        for (const BoxIndexEl &el : m_pillar_idx.query(plan_box(p.xy, p.xy, reach),
                                                       BoxIndex::qtIntersects)) {
            const PillarGeom &q = m_pillars[el.second];
            if (q.id == p.id || !q.braceable || built_pair(p.id, q.id)) continue;

            CandidateEdge e;
            e.a = std::min(p.id, q.id);
            e.b = std::max(p.id, q.id);
            e.dist = (pillar(e.a).xy - pillar(e.b).xy).norm();
            if (e.dist > reach) continue;
            out.emplace_back(e);
        }
    }

    std::sort(out.begin(), out.end(), [](const CandidateEdge &l, const CandidateEdge &r) {
        if (l.dist != r.dist) return l.dist < r.dist;
        if (l.a != r.a) return l.a < r.a;
        return l.b < r.b;
    });
    out.erase(std::unique(out.begin(), out.end(),
                          [](const CandidateEdge &l, const CandidateEdge &r) {
                              return l.a == r.a && l.b == r.b;
                          }),
              out.end());
    return out;
}

BracingPlanner::Evaluation BracingPlanner::evaluate(const PillarGeom &pa,
                                                    const PillarGeom &pb,
                                                    double dist, Mode mode) const
{
    Evaluation ev;
    const double c     = m_cfg.safety_clearance;
    const double r_bar = std::min(pa.r, pb.r);
    const bool   in_3d = mode == Mode::rescue || mode == Mode::fallback;

    // Pillars of one cluster are one pillar to every rule below: none is in
    // the way of another, and groups leaving it share it.
    const std::vector<size_t> ra = roots_of(pa), rb = roots_of(pb);
    auto at_ends = [&](long id) {
        const long root = root_of_id(id);
        return id == pa.id || id == pb.id || contains(ra, root) || contains(rb, root);
    };

    // 1. too_close
    if (dist < m_cfg.min_pillar_distance) { ev.reason = Reason::too_close; return ev; }

    // 2. pair_cap: none for props; the safety net counts structural links.
    if (mode == Mode::fallback) {
        if (pa.links >= m_cfg.pair_cap || pb.links >= m_cfg.pair_cap) {
            ev.reason = Reason::pair_cap;
            return ev;
        }
    } else if (mode != Mode::attach &&
               (pa.pairs >= m_cfg.pair_cap || pb.pairs >= m_cfg.pair_cap)) {
        ev.reason = Reason::pair_cap;
        return ev;
    }

    // 3. obstacle_pillar: every other pillar, full or frozen, at any height.
    for (const BoxIndexEl &el : m_pillar_idx.query(plan_box(pa.xy, pb.xy, r_bar + c),
                                                   BoxIndex::qtIntersects)) {
        const PillarGeom &q = m_pillars[el.second];
        if (at_ends(q.id)) continue;
        if (geom::point_seg_dist2d(q.xy, pa.xy, pb.xy) < r_bar + q.r + c) {
            ev.reason = Reason::obstacle_pillar;
            return ev;
        }
    }

    // 4. shared_angle: groups that share a pillar - or leave one cluster -
    //    are judged by angle only, each from its own pillar's axis.
    for (const PillarGeom *s : {&pa, &pb}) {
        const PillarGeom &other_end = s == &pa ? pb : pa;
        const Vec2d       dir       = other_end.xy - s->xy;
        for (size_t root : s == &pa ? ra : rb) {
            for (size_t mi : m_members.at(root)) {
                const PillarGeom &m  = m_pillars[mi];
                const auto        it = m_truss_of.find(m.id);
                if (it == m_truss_of.end()) continue;
                for (int ti : it->second) {
                    const BuiltTruss &t   = m_built[size_t(ti)];
                    const long        far = t.a == m.id ? t.b : t.a;
                    if (m.id == s->id && far == other_end.id) continue; // the same pair
                    if (!geom::dir_angle_ok(dir, pillar(far).xy - m.xy,
                                            m_cfg.cos_min_shared_ang)) {
                        ev.reason = Reason::shared_angle;
                        return ev;
                    }
                }
            }
        }
    }

    // 5. cross_truss: groups with no pillar in common, plan view. A rescue
    //    and the safety net replace it with the bar-by-bar 3D test below.
    if (!in_3d) {
        for (const BoxIndexEl &el : m_truss_idx.query(plan_box(pa.xy, pb.xy, r_bar + c),
                                                      BoxIndex::qtIntersects)) {
            const BuiltTruss &t = m_built[el.second];
            if (at_ends(t.a) || at_ends(t.b)) continue;
            if (geom::seg_seg_dist2d(pa.xy, pb.xy, pillar(t.a).xy, pillar(t.b).xy) <
                r_bar + t.r + c) {
                ev.reason = Reason::cross_truss;
                return ev;
            }
        }
    }

    // 6. bar by bar
    std::vector<Segment3> bars = geom::truss_bars(pa, pb, m_cfg.truss);
    if (bars.empty()) { ev.reason = Reason::no_room; return ev; }

    auto too_near = [c](const Segment3 &bar, const Segment3 &other) {
        const double lim = bar.r + other.r + c;
        // Plan view first: never misses a contact, and is cheap.
        return geom::seg_seg_dist2d(plan(bar.a), plan(bar.b), plan(other.a), plan(other.b)) < lim &&
               geom::seg_seg_dist3d(bar.a, bar.b, other.a, other.b) < lim;
    };

    bool any_z = false;
    for (const Segment3 &bar : bars) {
        const BoundingBox qb = plan_box(plan(bar.a), plan(bar.b), bar.r + c);
        bool z_fail = false;

        for (const BoxIndexEl &el : m_bar_idx.query(qb, BoxIndex::qtIntersects)) {
            const ObstacleBar &o = m_obstacles[el.second];
            // A bar ending on one of this group's pillars meets it there.
            if (at_ends(o.owner_a) || at_ends(o.owner_b)) continue;
            if (too_near(bar, o.seg)) { z_fail = true; break; }
        }

        if (!z_fail && in_3d) {
            for (const BoxIndexEl &el : m_truss_idx.query(qb, BoxIndex::qtIntersects)) {
                const BuiltTruss &t = m_built[el.second];
                if (at_ends(t.a) || at_ends(t.b)) continue;
                for (const Segment3 &tb : t.bars)
                    if (too_near(bar, tb)) { z_fail = true; break; }
                if (z_fail) break;
            }
        }

        if (z_fail) { any_z = true; continue; }
        if (m_mesh_clear && !m_mesh_clear(bar.a, bar.b, bar.r)) continue;
        ev.bars.push_back(bar);
    }

    if (ev.bars.empty()) ev.reason = any_z ? Reason::z_clear_fail : Reason::mesh_hit;
    return ev;
}

int BracingPlanner::accept(const CandidateEdge &e, std::vector<Segment3> &&bars, Mode mode)
{
    PillarGeom &pa = pillar_mut(e.a);
    PillarGeom &pb = pillar_mut(e.b);

    BuiltTruss t;
    t.a = e.a; t.b = e.b;
    t.r = std::min(pa.r, pb.r);
    t.bars = std::move(bars);
    t.rescued  = mode == Mode::rescue;
    t.fallback = mode == Mode::fallback;

    const int ti = int(m_built.size());
    m_truss_idx.insert(plan_box(pa.xy, pb.xy, t.r), unsigned(ti));
    m_built.emplace_back(std::move(t));
    m_truss_of[e.a].push_back(ti);
    m_truss_of[e.b].push_back(ti);

    // Every group counts as a pair; the structural count keeps its old rule.
    const bool link_a = gives_link(pa, pb);
    const bool link_b = gives_link(pb, pa);
    ++pa.pairs; ++pb.pairs;
    pa.links += link_a;
    pb.links += link_b;

    return ti;
}

std::vector<Decision> BracingPlanner::run_greedy()
{
    assert(!m_greedy_done);
    m_greedy_done = true;

    std::vector<long> ids;
    ids.reserve(m_pillars.size());
    for (const PillarGeom &p : m_pillars) ids.push_back(p.id);
    m_candidates = collect(ids);

    std::vector<Decision> out;
    out.reserve(m_candidates.size());
    for (const CandidateEdge &e : m_candidates) {
        Evaluation ev = evaluate(pillar(e.a), pillar(e.b), e.dist, Mode::greedy);
        Decision d;
        d.edge = e;
        d.reason = ev.reason;
        d.bars_built = unsigned(ev.bars.size());
        if (ev.reason == Reason::accepted) d.truss = accept(e, std::move(ev.bars), Mode::greedy);
        out.emplace_back(d);
    }
    return out;
}

std::vector<Decision> BracingPlanner::run_rescue()
{
    assert(m_greedy_done);
    std::vector<Decision> out;

    for (const CandidateEdge &e : m_candidates) {
        if (built_pair(e.a, e.b)) continue;
        // Judged afresh at every step: a pillar that has just been rescued is
        // not rescued a second time through another of its candidates.
        if (!rescue_target(pillar(e.a)) && !rescue_target(pillar(e.b))) continue;

        Evaluation ev = evaluate(pillar(e.a), pillar(e.b), e.dist, Mode::rescue);
        Decision d;
        d.edge = e;
        d.reason = ev.reason;
        d.bars_built = unsigned(ev.bars.size());
        d.rescue = true;
        if (ev.reason == Reason::accepted) d.truss = accept(e, std::move(ev.bars), Mode::rescue);
        out.emplace_back(d);
    }
    return out;
}

Decision BracingPlanner::try_attach(long main_id, const PillarGeom &prop) const
{
    // The main pillar goes first, as interconnect(pillar(), prop) had it; the
    // prop is given the next id, so attach() keeps that order.
    const PillarGeom &pm = pillar(main_id);
    Decision d;
    d.edge.a = main_id;
    d.edge.b = prop.id;
    d.edge.dist = (pm.xy - prop.xy).norm();

    if (d.edge.dist > m_cfg.max_link_distance) {
        d.reason = Reason::out_of_range;
        return d;
    }

    Evaluation ev = evaluate(pm, prop, d.edge.dist, Mode::attach);
    d.reason = ev.reason;
    d.bars_built = unsigned(ev.bars.size());
    return d;
}

Decision BracingPlanner::attach(long a, long b)
{
    Decision d;
    d.edge.a = std::min(a, b);
    d.edge.b = std::max(a, b);
    const PillarGeom &pa = pillar(d.edge.a), &pb = pillar(d.edge.b);
    d.edge.dist = (pa.xy - pb.xy).norm();

    if (d.edge.dist > m_cfg.max_link_distance) {
        d.reason = Reason::out_of_range;
        return d;
    }

    Evaluation ev = evaluate(pa, pb, d.edge.dist, Mode::attach);
    d.reason = ev.reason;
    d.bars_built = unsigned(ev.bars.size());
    if (ev.reason == Reason::accepted) d.truss = accept(d.edge, std::move(ev.bars), Mode::attach);
    return d;
}

std::vector<Decision> BracingPlanner::run_greedy_for(const std::vector<long> &new_ids)
{
    std::vector<Decision> out;
    for (const CandidateEdge &e : collect(new_ids)) {
        Evaluation ev = evaluate(pillar(e.a), pillar(e.b), e.dist, Mode::greedy);
        Decision d;
        d.edge = e;
        d.reason = ev.reason;
        d.bars_built = unsigned(ev.bars.size());
        if (ev.reason == Reason::accepted) d.truss = accept(e, std::move(ev.bars), Mode::greedy);
        out.emplace_back(d);
    }
    return out;
}

std::vector<Decision> BracingPlanner::run_fallback()
{
    std::vector<long> lonely;
    for (const PillarGeom &p : m_pillars)
        if (rescue_target(p)) lonely.push_back(p.id);

    std::vector<Decision> out;
    if (lonely.empty()) return out;

    // Collected afresh: the auxiliary stage may have added pillars since.
    for (const CandidateEdge &e : collect(lonely)) {
        if (built_pair(e.a, e.b)) continue;
        const PillarGeom &pa = pillar(e.a), &pb = pillar(e.b);
        // Judged afresh at every step, like the rescue; and only a pillar
        // that would give the lonely one a link is worth a group.
        const bool for_a = rescue_target(pa) && gives_link(pa, pb);
        const bool for_b = rescue_target(pb) && gives_link(pb, pa);
        if (!for_a && !for_b) continue;

        Evaluation ev = evaluate(pa, pb, e.dist, Mode::fallback);
        Decision d;
        d.edge = e;
        d.reason = ev.reason;
        d.bars_built = unsigned(ev.bars.size());
        d.fallback = true;
        if (ev.reason == Reason::accepted) d.truss = accept(e, std::move(ev.bars), Mode::fallback);
        out.emplace_back(d);
    }
    return out;
}

}}} // namespace Slic3r::sla::bracing
