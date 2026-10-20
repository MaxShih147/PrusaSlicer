#include "RoutePlan.hpp"

#include <cassert>

namespace Slic3r { namespace sla {

using Op = BuildStep::Op;

long BuildRecipe::add_pillar(long head_id, double length)
{
    BuildStep s{Op::head_pillar};
    s.head = head_id;
    s.len  = length;
    push(s);
    return LOCAL + m_pillars++;
}

long BuildRecipe::add_pillar(const Vec3d &endpt, double height, double r_start, double r_end)
{
    BuildStep s{Op::pillar};
    s.a   = endpt;
    s.len = height;
    s.r   = r_start;
    s.r2  = r_end;
    push(s);
    return LOCAL + m_pillars++;
}

void BuildRecipe::add_pillar_base(long pillar_id, double height, double radius)
{
    BuildStep s{Op::pillar_base};
    s.pillar = pillar_id;
    s.len    = height;
    s.r      = radius;
    push(s);
}

void BuildRecipe::add_junction(const Vec3d &pos, double r)
{
    BuildStep s{Op::junction};
    s.a = pos;
    s.r = r;
    push(s);
    ++m_junctions;
}

void BuildRecipe::own_junctions_from(size_t first, long pillar_id)
{
    BuildStep s{Op::own_junctions};
    s.ref    = long(first);
    s.pillar = pillar_id;
    push(s);
}

Bridge BuildRecipe::add_bridge(const Vec3d &st, const Vec3d &e, double r, BridgeKind kind,
                               long pillar_id)
{
    BuildStep s{Op::bridge};
    s.a      = st;
    s.b      = e;
    s.r      = r;
    s.kind   = kind;
    s.pillar = pillar_id;
    push(s);

    Bridge br{st, e, r};
    br.id      = LOCAL + m_bridges++;
    br.kind    = kind;
    br.owner_b = pillar_id;
    return br;
}

Bridge BuildRecipe::add_bridge_between(long pillar_id, const Vec3d &st, const Vec3d &e, double r,
                                       BridgeKind kind, long from_pillar)
{
    BuildStep s{Op::bridge_between};
    s.pillar = pillar_id;
    s.a      = st;
    s.b      = e;
    s.r      = r;
    s.kind   = kind;
    s.ref    = from_pillar;
    push(s);

    Bridge br{st, e, r};
    br.id      = LOCAL + m_bridges++;
    br.kind    = kind;
    br.owner_a = from_pillar;
    br.owner_b = pillar_id;
    return br;
}

void BuildRecipe::add_bridge_to_pillar(long head_id, const Vec3d &endp, long pillar_id)
{
    BuildStep s{Op::head_bridge};
    s.head   = head_id;
    s.b      = endp;
    s.pillar = pillar_id;
    push(s);
    ++m_bridges;
}

DiffBridge BuildRecipe::add_diffbridge(const DiffBridge &br)
{
    BuildStep s{Op::diffbridge};
    s.a  = br.startp;
    s.b  = br.endp;
    s.r  = br.r;
    s.r2 = br.end_r;
    push(s);

    DiffBridge out{br.startp, br.endp, br.r, br.end_r};
    out.id   = LOCAL + m_diffbridges++;
    out.kind = BridgeKind::route;
    return out;
}

void BuildRecipe::own_bridge(long bridge_id, long pillar_id)
{
    BuildStep s{Op::own_bridge};
    s.ref    = bridge_id;
    s.pillar = pillar_id;
    push(s);
}

void BuildRecipe::own_diffbridge(long bridge_id, long pillar_id)
{
    BuildStep s{Op::own_diffbridge};
    s.ref    = bridge_id;
    s.pillar = pillar_id;
    push(s);
}

void BuildRecipe::set_head_bridge(long head_id, long bridge_id)
{
    BuildStep s{Op::head_bridge_id};
    s.head = head_id;
    s.ref  = bridge_id;
    push(s);
}

void BuildRecipe::set_head_diffbridge(long head_id, long bridge_id)
{
    BuildStep s{Op::head_diffbridge_id};
    s.head = head_id;
    s.ref  = bridge_id;
    push(s);
}

void BuildRecipe::set_head_pillar(long head_id, long pillar_id)
{
    BuildStep s{Op::head_pillar_id};
    s.head   = head_id;
    s.pillar = pillar_id;
    push(s);
}

void BuildRecipe::add_anchor(double r_back, double r_pin, double width, double penetration,
                             const Vec3d &dir, const Vec3d &pos)
{
    BuildStep s{Op::anchor};
    s.r   = r_back;
    s.r2  = r_pin;
    s.len = width;
    s.pen = penetration;
    s.b   = dir;
    s.a   = pos;
    push(s);
}

void BuildRecipe::increment_bridges(long pillar_id)
{
    BuildStep s{Op::increment_bridges};
    s.pillar = pillar_id;
    push(s);
}

RecipeIds replay(SupportTreeBuilder &builder, const BuildRecipe &recipe)
{
    RecipeIds ids;

    // A recipe-local id to the builder's; any other id is the builder's already.
    auto real = [](const std::vector<long> &made, long id) {
        if (!BuildRecipe::is_local(id))
            return id;
        assert(size_t(id - BuildRecipe::LOCAL) < made.size());
        return made[size_t(id - BuildRecipe::LOCAL)];
    };
    auto P = [&](long id) { return real(ids.pillars, id); };
    auto B = [&](long id) { return real(ids.bridges, id); };
    auto D = [&](long id) { return real(ids.diffbridges, id); };

    for (const BuildStep &s : recipe.steps()) {
        switch (s.op) {
        case Op::head_pillar:
            ids.pillars.push_back(builder.add_pillar(s.head, s.len));
            break;
        case Op::pillar:
            ids.pillars.push_back(builder.add_pillar(s.a, s.len, s.r, s.r2));
            break;
        case Op::pillar_base:
            builder.add_pillar_base(P(s.pillar), s.len, s.r);
            break;
        case Op::junction:
            ids.junctions.push_back(builder.add_junction(s.a, s.r).id);
            break;
        case Op::own_junctions: {
            // From the first junction the recipe made at or after that point;
            // none made since means none to own, as on the builder.
            const size_t k = size_t(s.ref - BuildRecipe::LOCAL);
            const size_t first = k < ids.junctions.size() ? size_t(ids.junctions[k])
                                                          : builder.junctioncount();
            builder.own_junctions_from(first, P(s.pillar));
            break;
        }
        case Op::bridge:
            ids.bridges.push_back(builder.add_bridge(s.a, s.b, s.r, s.kind, P(s.pillar)).id);
            break;
        case Op::bridge_between:
            ids.bridges.push_back(
                builder.add_bridge_between(P(s.pillar), s.a, s.b, s.r, s.kind, P(s.ref)).id);
            break;
        case Op::head_bridge:
            ids.bridges.push_back(builder.add_bridge_to_pillar(s.head, s.b, P(s.pillar)).id);
            break;
        case Op::diffbridge:
            ids.diffbridges.push_back(builder.add_diffbridge(DiffBridge{s.a, s.b, s.r, s.r2}).id);
            break;
        case Op::own_bridge:
            builder.own_bridge(B(s.ref), P(s.pillar));
            break;
        case Op::own_diffbridge:
            builder.own_diffbridge(D(s.ref), P(s.pillar));
            break;
        case Op::head_bridge_id:
            builder.head(unsigned(s.head)).bridge_id = B(s.ref);
            break;
        case Op::head_diffbridge_id:
            builder.head(unsigned(s.head)).bridge_id = D(s.ref);
            break;
        case Op::head_pillar_id:
            builder.head(unsigned(s.head)).pillar_id = P(s.pillar);
            break;
        case Op::anchor:
            builder.add_anchor(s.r, s.r2, s.len, s.pen, s.b, s.a);
            break;
        case Op::increment_bridges:
            builder.increment_bridges(builder.pillar(P(s.pillar)));
            break;
        }
    }

    return ids;
}

}} // namespace Slic3r::sla
