#include "BridgeClearance.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iterator>
#include <limits>
#include <utility>

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <libslic3r/SLA/BracingPlanner.hpp>

namespace Slic3r { namespace sla { namespace clearance {

namespace bg  = boost::geometry;
namespace bgi = boost::geometry::index;

const char *kind_name(PrimKind k)
{
    switch (k) {
    case PrimKind::head_bar:  return "head_bar";
    case PrimKind::route_bar: return "route_bar";
    case PrimKind::prop_bar:  return "prop_bar";
    case PrimKind::pillar:    return "pillar";
    case PrimKind::pinhead:   return "pinhead";
    }
    return "?";
}

// ─── elements to capsules ────────────────────────────────────────────────────

Capsule bar_capsule(const Vec3d &startp, const Vec3d &endp, double r)
{
    return {startp, endp, r};
}

Capsule diffbridge_capsule(const Vec3d &startp, const Vec3d &endp, double r, double end_r)
{
    return {startp, endp, std::max(r, end_r)};
}

Capsule pillar_capsule(const Vec3d &endpt, double height, double r_start, double r_end)
{
    return {endpt, Vec3d{endpt.x(), endpt.y(), endpt.z() + height}, std::max(r_start, r_end)};
}

Capsule pinhead_capsule(const Vec3d &pos, const Vec3d &junction, double r_back)
{
    return {pos, junction, r_back};
}

// ─── distances ───────────────────────────────────────────────────────────────

double capsule_dist(const Capsule &x, const Capsule &y)
{
    return bracing::geom::seg_seg_dist3d(x.a, x.b, y.a, y.b);
}

double joint_radius(double r_pillar, double r1, double r2)
{
    return r_pillar + r1 + r2 + SAFETY_CLEARANCE;
}

bool cylinder_interval(const Vec3d &a, const Vec3d &b, const Vec2d &axis, double R,
                       double &t0, double &t1)
{
    const Vec2d  d = b.head<2>() - a.head<2>(), w = a.head<2>() - axis;
    const double A = d.squaredNorm(), R2 = R * R;

    if (A <= bracing::geom::EPS_LEN) {
        // A point in plan view, by the same test seg_seg_dist3d() uses: the
        // quadratic below degenerates. Judged by the ends, the squared plan
        // distance taken as linear in between (its curvature A is below
        // EPS_LEN), so the result does not jump at the threshold.
        const double f0 = w.squaredNorm() - R2, f1 = (b.head<2>() - axis).squaredNorm() - R2;
        if (f0 <= 0. && f1 <= 0.) { t0 = 0.; t1 = 1.; return true; }
        if (f0 > 0. && f1 > 0.) return false;
        const double tc = f0 / (f0 - f1); // the ends differ in sign, so f0 != f1
        if (f0 <= 0.) { t0 = 0.; t1 = tc; }
        else          { t0 = tc; t1 = 1.; }
        return t0 < t1;
    }

    // |w + t d|^2 = R^2  <=>  A t^2 + 2 B t + C = 0
    const double B = w.dot(d), C = w.squaredNorm() - R2;
    const double disc = B * B - A * C;
    if (disc <= 0.) return false; // misses, or only touches the surface

    // The root that does not subtract nearly equal numbers, and the other one
    // from the product of the roots, C / A. q != 0, as disc > 0.
    const double q  = -(B + std::copysign(std::sqrt(disc), B));
    const double ra = q / A, rb = C / q;
    t0 = std::max(std::min(ra, rb), 0.);
    t1 = std::min(std::max(ra, rb), 1.);
    return t0 < t1;
}

std::vector<Piece> clip_joints(const Capsule &x, double r_other,
                               const std::vector<JointAxis> &shared)
{
    std::vector<Piece> pieces{Piece{0., 1.}};

    for (const JointAxis &P : shared) {
        double t0, t1;
        if (!cylinder_interval(x.a, x.b, P.xy, joint_radius(P.r, x.r, r_other), t0, t1))
            continue;

        std::vector<Piece> left;
        for (const Piece &pc : pieces) {
            // Subtract the closed [t0, t1]; what stays keeps its ends on the
            // surface (closure), and a mere point of it is dropped.
            const double lo_end = std::min(pc.t1, t0), hi_start = std::max(pc.t0, t1);
            if (lo_end - pc.t0 > EPS_PIECE) left.push_back({pc.t0, lo_end});
            if (pc.t1 - hi_start > EPS_PIECE) left.push_back({hi_start, pc.t1});
        }
        pieces = std::move(left);
        if (pieces.empty()) break;
    }

    return pieces;
}

namespace {

// The ends are taken as they are, so an unclipped piece is exactly the
// capsule's own axis and its distance is exactly capsule_dist().
Vec3d point_at(const Capsule &c, double t)
{
    if (t <= 0.) return c.a;
    if (t >= 1.) return c.b;
    return c.a + t * (c.b - c.a);
}

Vec3d point_on(const Vec3d &p, const Vec3d &q, double s) { return p + s * (q - p); }

// Distance of the closest pair of pieces, and the closest points; +infinity
// with no pieces on either side.
double closest_pieces(const Capsule &x, const std::vector<Piece> &px, const Capsule &y,
                      const std::vector<Piece> &py, Vec3d &on_x, Vec3d &on_y)
{
    double best = std::numeric_limits<double>::infinity();
    for (const Piece &i : px) {
        const Vec3d xa = point_at(x, i.t0), xb = point_at(x, i.t1);
        for (const Piece &j : py) {
            const Vec3d ya = point_at(y, j.t0), yb = point_at(y, j.t1);
            double s, t;
            const double d = bracing::geom::seg_seg_closest3d(xa, xb, ya, yb, s, t);
            if (d < best) {
                best = d;
                on_x = point_on(xa, xb, s);
                on_y = point_on(ya, yb, t);
            }
        }
    }
    return best;
}

double joint_clipped(const Capsule &x, const Capsule &y, const std::vector<JointAxis> &shared,
                     Vec3d &on_x, Vec3d &on_y)
{
    const std::vector<Piece> px = clip_joints(x, y.r, shared);
    if (px.empty()) return std::numeric_limits<double>::infinity();
    const std::vector<Piece> py = clip_joints(y, x.r, shared);
    if (py.empty()) return std::numeric_limits<double>::infinity();
    return closest_pieces(x, px, y, py, on_x, on_y);
}

bool is_bar(PrimKind k)
{
    return k == PrimKind::head_bar || k == PrimKind::route_bar || k == PrimKind::prop_bar;
}

// The pillars an element stands on, ascending, without -1 or repeats.
std::vector<long> pillar_set(const Prim &p)
{
    std::vector<long> s;
    if (p.pillar_a >= 0) s.push_back(p.pillar_a);
    if (is_bar(p.kind) && p.pillar_b >= 0 && p.pillar_b != p.pillar_a) s.push_back(p.pillar_b);
    std::sort(s.begin(), s.end());
    return s;
}

// x is a bar ending on pillar y, or a pinhead carried by pillar y.
bool rests_on(const Prim &x, const Prim &y)
{
    if (y.kind != PrimKind::pillar || y.pillar_a < 0) return false;
    if (is_bar(x.kind)) return y.pillar_a == x.pillar_a || y.pillar_a == x.pillar_b;
    if (x.kind == PrimKind::pinhead) return y.pillar_a == x.pillar_a;
    return false;
}

} // namespace

double joint_clipped_dist(const Capsule &x, const Capsule &y,
                          const std::vector<JointAxis> &shared)
{
    Vec3d on_x, on_y;
    return joint_clipped(x, y, shared, on_x, on_y);
}

// ─── the rules between two elements ──────────────────────────────────────────

bool judged_kinds(PrimKind x, PrimKind y)
{
    if (x == PrimKind::pillar && y == PrimKind::pillar) return false;
    if (x == PrimKind::pinhead && y == PrimKind::pinhead) return false;
    return true;
}

bool exempt(const Prim &x, const Prim &y)
{
    if (x.head_id >= 0 && x.head_id == y.head_id) return true;
    return rests_on(x, y) || rests_on(y, x);
}

std::vector<long> shared_pillars(const Prim &x, const Prim &y)
{
    const std::vector<long> sx = pillar_set(x), sy = pillar_set(y);
    std::vector<long> s;
    std::set_intersection(sx.begin(), sx.end(), sy.begin(), sy.end(), std::back_inserter(s));
    return s;
}

bool pair_contact(const Prim &cand, const Prim &old, double c, const PillarAxisFn &axis_of,
                  Contact &out)
{
    if (!judged_kinds(cand.kind, old.kind) || exempt(cand, old)) return false;

    const Capsule &x = cand.cap, &y = old.cap;
    const double required = x.r + y.r + c;

    // Clipping only takes parts away, so it can only lengthen the distance: a
    // pair clear in full is clear, and most broad-phase hits end here.
    double s, t;
    double d = bracing::geom::seg_seg_closest3d(x.a, x.b, y.a, y.b, s, t);
    if (!(d < required)) return false; // exactly at the limit is clear
    Vec3d on_new = point_on(x.a, x.b, s), on_old = point_on(y.a, y.b, t);

    const std::vector<long> ids = shared_pillars(cand, old);
    if (!ids.empty()) {
        std::vector<JointAxis> shared;
        shared.reserve(ids.size());
        for (long id : ids) shared.push_back(axis_of(id));

        d = joint_clipped(x, y, shared, on_new, on_old);
        if (!(d < required)) return false;
    }

    out = Contact{};
    out.p_new = on_new;
    out.p_old = on_old;
    out.dist = d;
    out.required = required;
    out.with = old.kind;
    out.with_head = old.head_id;
    // A head bar starts at its head; the pillar it belongs to is at its end.
    out.with_pillar = is_bar(old.kind) && old.pillar_b >= 0 ? old.pillar_b : old.pillar_a;
    return true;
}

// ─── 3D index ────────────────────────────────────────────────────────────────

namespace {

using Point3 = bg::model::point<double, 3, bg::cs::cartesian>;
using Box3   = bg::model::box<Point3>;
using Value  = std::pair<Box3, unsigned>;

// The capsule's bounding box grown by `grow` on every side.
Box3 box_of(const Capsule &c, double grow)
{
    const Vec3d lo = (c.a.cwiseMin(c.b).array() - grow).matrix();
    const Vec3d hi = (c.a.cwiseMax(c.b).array() + grow).matrix();
    return {Point3{lo.x(), lo.y(), lo.z()}, Point3{hi.x(), hi.y(), hi.z()}};
}

} // namespace

class ClearanceIndex::Impl
{
public:
    bgi::rtree<Value, bgi::rstar<16, 4>> tree;
    std::vector<Prim> prims;
};

ClearanceIndex::ClearanceIndex() : m_impl(std::make_unique<Impl>()) {}
ClearanceIndex::~ClearanceIndex() = default;
ClearanceIndex::ClearanceIndex(ClearanceIndex &&) noexcept = default;
ClearanceIndex &ClearanceIndex::operator=(ClearanceIndex &&) noexcept = default;

size_t ClearanceIndex::insert(const Prim &p)
{
    const size_t idx = m_impl->prims.size();
    m_impl->prims.push_back(p);
    // Stored grown by its own radius; a query adds the other radius and c.
    m_impl->tree.insert(Value{box_of(p.cap, p.cap.r), unsigned(idx)});
    return idx;
}

const std::vector<Prim> &ClearanceIndex::prims() const { return m_impl->prims; }

size_t ClearanceIndex::size() const { return m_impl->prims.size(); }

std::vector<Contact> ClearanceIndex::violations(const std::vector<Prim> &cands, double c,
                                                const PillarAxisFn &axis_of) const
{
    assert(c >= 0.);
    std::vector<Contact> out;
    std::vector<Value>   hits;

    for (size_t i = 0; i < cands.size(); ++i) {
        const Prim &cand = cands[i];
        // Two axes closer than r1 + r2 + c have points that close; those lie
        // in the unexpanded boxes, so these boxes meet: a superset. EPS_BROAD
        // keeps it one when the box edges and the limit round differently.
        hits.clear();
        m_impl->tree.query(bgi::intersects(box_of(cand.cap, cand.cap.r + c + EPS_BROAD)),
                           std::back_inserter(hits));

        for (const Value &v : hits) {
            Contact ct;
            if (!pair_contact(cand, m_impl->prims[v.second], c, axis_of, ct)) continue;
            ct.cand = i;
            ct.with_index = v.second;
            out.push_back(ct);
        }
    }

    // The tree's order is its own; the caller sees one fixed order.
    std::sort(out.begin(), out.end(), [](const Contact &l, const Contact &r) {
        return l.cand != r.cand ? l.cand < r.cand : l.with_index < r.with_index;
    });
    return out;
}

}}} // namespace Slic3r::sla::clearance
