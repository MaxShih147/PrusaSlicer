#include <catch2/catch_test_macros.hpp>
#include <test_utils.hpp>

#include <climits>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include <libslic3r/ExPolygon.hpp>
#include <libslic3r/BoundingBox.hpp>
#include <libslic3r/SLA/SpatIndex.hpp>
#include <libslic3r/ClipperUtils.hpp>
#include <libslic3r/TriangleMeshSlicer.hpp>

#include <libslic3r/SLA/SupportIslands/SampleConfigFactory.hpp>
#include <libslic3r/SLA/SupportIslands/SampleConfig.hpp>
#include <libslic3r/SLA/SupportIslands/VoronoiGraphUtils.hpp>
#include <libslic3r/SLA/SupportIslands/UniformSupportIsland.hpp>
#include <libslic3r/SLA/SupportIslands/PolygonUtils.hpp>
#include <libslic3r/EngineErrorCodes.hpp>
#include "nanosvg/nanosvg.h"    // load SVG file
#include "sla_test_utils.hpp"

using namespace Slic3r;
using namespace Slic3r::sla;

//#define STORE_SAMPLE_INTO_SVG_FILES "C:/data/temp/test_islands/sample_"
//#define STORE_ISLAND_ISSUES "C:/data/temp/issues/"

TEST_CASE("Overhanging point should be supported", "[SupGen]") {

    // Pyramid with 45 deg slope
    TriangleMesh mesh = make_pyramid(10.f, 10.f);
    mesh.rotate_y(float(PI));
    //mesh.WriteOBJFile("Pyramid.obj");

    sla::SupportPoints pts = calc_support_pts(mesh);

    // The overhang, which is the upside-down pyramid's edge
    Vec3f overh{0., 0., -10.};

    REQUIRE(!pts.empty());

    float dist = (overh - pts.front().pos).norm();

    for (const auto &pt : pts)
        dist = std::min(dist, (overh - pt.pos).norm());

    // Should require exactly one support point at the overhang
    REQUIRE(pts.size() > 0);
    REQUIRE(dist < 1.f);
}

double min_point_distance(const sla::SupportPoints &pts)
{
    sla::PointIndex index;

    for (size_t i = 0; i < pts.size(); ++i)
        index.insert(pts[i].pos.cast<double>(), i);

    auto d = std::numeric_limits<double>::max();
    index.foreach([&d, &index](const sla::PointIndexEl &el) {
        auto res = index.nearest(el.first, 2);
        for (const sla::PointIndexEl &r : res)
            if (r.second != el.second)
                d = std::min(d, (el.first - r.first).norm());
    });

    return d;
}

TEST_CASE("Overhanging horizontal surface should be supported", "[SupGen]") {
    double width = 10., depth = 10., height = 1.;

    TriangleMesh mesh = make_cube(width, depth, height); 
    mesh.translate(0., 0., 5.); // lift up
    // mesh.WriteOBJFile("Cuboid.obj");
    sla::SupportPoints pts = calc_support_pts(mesh);
    REQUIRE(!pts.empty());
}

template<class M> auto&& center_around_bb(M &&mesh)
{
    auto bb = mesh.bounding_box();
    mesh.translate(-bb.center().template cast<float>());

    return std::forward<M>(mesh);
}

TEST_CASE("Overhanging edge should be supported", "[SupGen]") {
    float width = 10.f, depth = 10.f, height = 5.f;

    TriangleMesh mesh = make_prism(width, depth, height);
    mesh.rotate_y(float(PI)); // rotate on its back
    mesh.translate(0., 0., height);
    mesh.WriteOBJFile("Prism.obj");

    sla::SupportPoints pts = calc_support_pts(mesh);

    Linef3 overh{ {0.f, -depth / 2.f, 0.f}, {0.f, depth / 2.f, 0.f}};

    // Get all the points closer that 1 mm to the overhanging edge:
    sla::SupportPoints overh_pts; overh_pts.reserve(pts.size());

    std::copy_if(pts.begin(), pts.end(), std::back_inserter(overh_pts),
                 [&overh](const sla::SupportPoint &pt){
                     return line_alg::distance_to(overh, Vec3d{pt.pos.cast<double>()}) < 1.;
                 });

    //double ddiff = min_point_distance(pts) - cfg.minimal_distance;
    //REQUIRE(ddiff > - 0.1 * cfg.minimal_distance);
}

TEST_CASE("Hollowed cube should be supported from the inside", "[SupGen][Hollowed]") {
    TriangleMesh mesh = make_cube(20., 20., 20.);

    hollow_mesh(mesh, HollowingConfig{});

    mesh.WriteOBJFile("cube_hollowed.obj");

    auto bb = mesh.bounding_box();
    auto h  = float(bb.max.z() - bb.min.z());
    Vec3f mv = bb.center().cast<float>() - Vec3f{0.f, 0.f, 0.5f * h};
    mesh.translate(-mv);

    sla::SupportPoints pts = calc_support_pts(mesh);
    //sla::remove_bottom_points(pts, mesh.bounding_box().min.z() + EPSILON);

    REQUIRE(!pts.empty());
}

TEST_CASE("Two parallel plates should be supported", "[SupGen][Hollowed]")
{
    double width = 20., depth = 20., height = 1.;

    TriangleMesh mesh = center_around_bb(make_cube(width + 5., depth + 5., height));
    TriangleMesh mesh_high = center_around_bb(make_cube(width, depth, height));
    mesh_high.translate(0., 0., 10.); // lift up
    mesh.merge(mesh_high);

    mesh.WriteOBJFile("parallel_plates.obj");

    sla::SupportPoints pts = calc_support_pts(mesh);
    //sla::remove_bottom_points(pts, mesh.bounding_box().min.z() + EPSILON);

    REQUIRE(!pts.empty());
}


Slic3r::Polygon create_cross_roads(double size, double width)
{
    auto r1 = PolygonUtils::create_rect(5.3 * size, width);
    r1.rotate(3.14/4);
    r1.translate(2 * size, width / 2);
    auto r2 = PolygonUtils::create_rect(6.1 * size, 3 / 4. * width);
    r2.rotate(-3.14 / 5);
    r2.translate(3 * size, width / 2);
    auto r3 = PolygonUtils::create_rect(7.9 * size, 4 / 5. * width);
    r3.translate(2*size, width/2);
    auto r4 = PolygonUtils::create_rect(5 / 6. * width, 5.7 * size);
    r4.translate(-size,3*size);
    Polygons rr = union_(Polygons({r1, r2, r3, r4}));
    return rr.front();
}

ExPolygon create_trinagle_with_hole(double size)
{
    auto hole = PolygonUtils::create_equilateral_triangle(size / 3);
    hole.reverse();
    hole.rotate(3.14 / 4);
    return ExPolygon(PolygonUtils::create_equilateral_triangle(size), hole);
}

ExPolygon create_square_with_hole(double size, double hole_size)
{
    assert(sqrt(hole_size *hole_size / 2) < size);
    auto hole = PolygonUtils::create_square(hole_size);
    hole.rotate(M_PI / 4.); // 45
    hole.reverse();
    return ExPolygon(PolygonUtils::create_square(size), hole);
}

ExPolygon create_square_with_4holes(double size, double hole_size) {
    auto hole = PolygonUtils::create_square(hole_size);
    hole.reverse();
    double size_4 = size / 4;
    auto h1 = hole;
    h1.translate(size_4, size_4);
    auto h2 = hole;
    h2.translate(-size_4, size_4);
    auto h3   = hole;
    h3.translate(size_4, -size_4);
    auto h4   = hole;
    h4.translate(-size_4, -size_4);
    ExPolygon result(PolygonUtils::create_square(size));
    result.holes = Polygons({h1, h2, h3, h4});
    return result;
}

// boudary of circle
ExPolygon create_disc(double radius, double width, size_t count_line_segments)
{
    double width_2 = width / 2;
    auto   hole    = PolygonUtils::create_circle(radius - width_2,
                                            count_line_segments);
    hole.reverse();
    return ExPolygon(PolygonUtils::create_circle(radius + width_2,
                                                 count_line_segments),
                     hole);
}

Slic3r::Polygon create_V_shape(double height, double line_width, double angle = M_PI/4) {
    double angle_2 = angle / 2;
    auto   left_side  = PolygonUtils::create_rect(line_width, height);
    auto   right_side = left_side;
    right_side.rotate(-angle_2);
    double small_move = cos(angle_2) * line_width / 2;
    double side_move  = sin(angle_2) * height / 2 + small_move;
    right_side.translate(side_move,0);
    left_side.rotate(angle_2);
    left_side.translate(-side_move, 0);
    auto bottom = PolygonUtils::create_rect(4 * small_move, line_width);
    bottom.translate(0., -cos(angle_2) * height / 2 + line_width/2);
    Polygons polygons = union_(Polygons({left_side, right_side, bottom}));
    return polygons.front();
}

ExPolygon create_tiny_wide_test_1(double wide, double tiny)
{
    double hole_size = wide;
    double width     = 2 * wide + hole_size;
    double height    = wide + hole_size + tiny;
    auto   outline   = PolygonUtils::create_rect(width, height);
    auto   hole      = PolygonUtils::create_rect(hole_size, hole_size);
    hole.reverse();
    int hole_move_y = height/2 - (hole_size/2 + tiny);
    hole.translate(0, hole_move_y);
    
    ExPolygon result(outline);
    result.holes = {hole};
    return result;
}

ExPolygon create_tiny_wide_test_2(double wide, double tiny)
{
    double hole_size = wide;
    double width     = (3 + 1) * wide + 3 * hole_size;
    double height    = 2*wide + 2*tiny + 3*hole_size;
    auto outline = PolygonUtils::create_rect( width, height);
    auto   hole      = PolygonUtils::create_rect(hole_size, hole_size);
    hole.reverse();
    auto  hole2 = hole;// copy
    auto  hole3       = hole; // copy
    auto  hole4       = hole; // copy

    int   hole_move_x = wide + hole_size;
    int   hole_move_y = wide + hole_size;
    hole.translate(hole_move_x, hole_move_y);
    hole2.translate(-hole_move_x, hole_move_y);
    hole3.translate(hole_move_x, -hole_move_y);
    hole4.translate(-hole_move_x, -hole_move_y);

    auto hole5 = PolygonUtils::create_circle(hole_size / 2, 16);
    hole5.reverse();
    auto hole6 = hole5; // copy
    hole5.translate(0, hole_move_y);
    hole6.translate(0, -hole_move_y);

    auto hole7 = PolygonUtils::create_equilateral_triangle(hole_size);
    hole7.reverse();
    auto hole8 = PolygonUtils::create_circle(hole_size/2, 7, Point(hole_move_x,0));
    hole8.reverse();
    auto hole9 = PolygonUtils::create_circle(hole_size/2, 5, Point(-hole_move_x,0));
    hole9.reverse();

    ExPolygon result(outline);
    result.holes = {hole, hole2, hole3, hole4, hole5, hole6, hole7, hole8, hole9};
    return result;
}

ExPolygon create_tiny_between_holes(double wide, double tiny)
{
    double hole_size = wide;
    double width     = 2 * wide + 2*hole_size + tiny;
    double height    = 2 * wide + hole_size;
    auto   outline   = PolygonUtils::create_rect(width, height);
    auto   holeL      = PolygonUtils::create_rect(hole_size, hole_size);
    holeL.reverse();
    auto holeR       = holeL;
    int hole_move_x = (hole_size + tiny)/2;
    holeL.translate(-hole_move_x, 0);
    holeR.translate(hole_move_x, 0);

    ExPolygon result(outline);
    result.holes = {holeL, holeR};
    return result;
}

// stress test for longest path
// needs reshape
ExPolygon create_mountains(double size) {
    return ExPolygon({{0., 0.},
                      {size, 0.},
                      {5 * size / 6, size},
                      {4 * size / 6, size / 6},
                      {3 * size / 7, 2 * size},
                      {2 * size / 7, size / 6},
                      {size / 7, size}});
}

/// Neighbor points create trouble for voronoi - test of neccessary offseting(closing) of contour
ExPolygon create_cylinder_bottom_slice() {
    indexed_triangle_set its_cylinder = its_make_cylinder(6.6551999999999998, 11.800000000000001);
    MeshSlicingParams param;
    Polygons polygons = slice_mesh(its_cylinder, 0.0125000002, param);
    return ExPolygon{polygons.front()};
}

ExPolygon load_frog(){
    TriangleMesh mesh = load_model("frog_legs.obj");
    std::vector<ExPolygons> slices = slice_mesh_ex(mesh.its, {0.1f});
    return slices.front()[1];
}

ExPolygon load_svg(const std::string& svg_filepath) {
    struct NSVGimage *image = nsvgParseFromFile(svg_filepath.c_str(), "px", 96);
    ScopeGuard sg_image([&image] { nsvgDelete(image); });

    auto to_polygon = [](NSVGpath *path) { 
        Polygon r;
        r.points.reserve(path->npts);
        for (int i = 0; i < path->npts; i++)
            r.points.push_back(Point(path->pts[2 * i], path->pts[2 * i + 1]));
        return r;
    };

    for (NSVGshape *shape_ptr = image->shapes; shape_ptr != NULL; shape_ptr = shape_ptr->next) {
        const NSVGshape &shape = *shape_ptr;
        if (!(shape.flags & NSVG_FLAGS_VISIBLE)) continue; // is visible
        if (shape.fill.type != NSVG_PAINT_NONE) continue; // is not used fill
        if (shape.stroke.type == NSVG_PAINT_NONE) continue; // exist stroke
        //if (shape.strokeWidth < 1e-5f) continue; // is visible stroke width
        //if (shape.stroke.color != 4278190261) continue; // is red
        ExPolygon result;
        for (NSVGpath *path = shape.paths; path != NULL; path = path->next) {
            // Path order is reverse to path in file
            if (path->next == NULL) // last path is contour
                result.contour = to_polygon(path);
            else
                result.holes.push_back(to_polygon(path));        
        }
        return result;
    }
    REQUIRE(false);
    return {};
}

ExPolygons createTestIslands(double size)
{
    std::string dir = std::string(TEST_DATA_DIR PATH_SEPARATOR) + "sla_islands/";
    bool useFrogLeg = false;    
    // need post reorganization of longest path
    ExPolygons result = {
        // one support point
        ExPolygon(PolygonUtils::create_equilateral_triangle(size)), 
        ExPolygon(PolygonUtils::create_square(size)),
        ExPolygon(PolygonUtils::create_rect(size / 2, size)),
        ExPolygon(PolygonUtils::create_isosceles_triangle(size / 2, 3 * size / 2)), // small sharp triangle
        ExPolygon(PolygonUtils::create_circle(size / 2, 10)),
        create_square_with_4holes(size, size / 4),
        create_disc(size/4, size / 4, 10),
        ExPolygon(create_V_shape(2*size/3, size / 4)),

        // two support points
        ExPolygon(PolygonUtils::create_isosceles_triangle(size / 2, 3 * size)), // small sharp triangle
        ExPolygon(PolygonUtils::create_rect(size / 2, 3 * size)),
        ExPolygon(create_V_shape(1.5*size, size/3)),

        // tiny line support points
        ExPolygon(PolygonUtils::create_rect(size / 2, 10 * size)), // long line
        ExPolygon(create_V_shape(size*4, size / 3)),
        ExPolygon(create_cross_roads(size, size / 3)),
        create_disc(3*size, size / 4, 30),
        create_disc(2*size, size, 12), // 3 points
        create_square_with_4holes(5 * size, 5 * size / 2 - size / 3),

        // Tiny and wide part together with holes
        ExPolygon(PolygonUtils::create_isosceles_triangle(5. * size, 40. * size)),
        create_tiny_wide_test_1(3 * size, 2 / 3. * size),
        create_tiny_wide_test_2(3 * size, 2 / 3. * size),
        create_tiny_between_holes(3 * size, 2 / 3. * size),

        ExPolygon(PolygonUtils::create_equilateral_triangle(scale_(18.6))),
        create_cylinder_bottom_slice(),
        load_svg(dir + "lm_issue.svg"), // change from thick to thin and vice versa on circle
        load_svg(dir + "SPE-2674.svg"), // center of longest path lay inside of the VD node
        load_svg(dir + "SPE-2674_2.svg"), // missing Voronoi vertex even after the rotation of input.

        // still problem
        // three support points
        ExPolygon(PolygonUtils::create_equilateral_triangle(3 * size)), 
        ExPolygon(PolygonUtils::create_circle(size, 20)),

        create_mountains(size),
        create_trinagle_with_hole(size),
        create_square_with_hole(size, size / 2),
        create_square_with_hole(size, size / 3)
    };    
    if (useFrogLeg)
        result.push_back(load_frog());
    return result;
}

Points createNet(const BoundingBox& bounding_box, double distance)
{ 
    Point  size       = bounding_box.size();
    double distance_2 = distance / 2;
    int    cols1 = static_cast<int>(floor(size.x() / distance))+1;
    int    cols2 = static_cast<int>(floor((size.x() - distance_2) / distance))+1;
    // equilateral triangle height with side distance
    double h      = sqrt(distance * distance - distance_2 * distance_2);
    int    rows   = static_cast<int>(floor(size.y() / h)) +1;
    int    rows_2 = rows / 2;
    size_t count_points = rows_2 * (cols1 + static_cast<size_t>(cols2));
    if (rows % 2 == 1) count_points += cols2;
    Points result;
    result.reserve(count_points);
    bool   isOdd = true;
    Point offset = bounding_box.min;
    double x_max = offset.x() + static_cast<double>(size.x());
    double y_max  = offset.y() + static_cast<double>(size.y());
    for (double y = offset.y(); y <= y_max; y += h) {
        double x_offset = offset.x();
        if (isOdd) x_offset += distance_2;
        isOdd = !isOdd;
        for (double x = x_offset; x <= x_max; x += distance) {
            result.emplace_back(x, y);
        }
    }
    assert(result.size() == count_points);
    return result; 
}

// create uniform triangle net and return points laying inside island
Points rasterize(const ExPolygon &island, double distance) {
    BoundingBox bb;
    for (const Point &pt : island.contour.points) bb.merge(pt);
    Points      fullNet = createNet(bb, distance);
    Points result;
    result.reserve(fullNet.size());
    std::copy_if(fullNet.begin(), fullNet.end(), std::back_inserter(result),
                 [&island](const Point &p) { return island.contains(p); });
    return result;
}

SupportIslandPoints test_island_sampling(const ExPolygon &   island,
                                        const SampleConfig &config)
{
    auto points = uniform_support_island(island, {}, config);

    Points chck_points = rasterize(island, config.head_radius); // TODO: Use resolution of printer
    bool is_island_supported = true; // Check rasterized island points that exist support point in max_distance
    double max_distance = config.thick_inner_max_distance;
    std::vector<double> point_distances(chck_points.size(), {max_distance + 1});
    for (size_t index = 0; index < chck_points.size(); ++index) { 
        const Point &chck_point  = chck_points[index];
        double &min_distance = point_distances[index];
        bool         exist_close_support_point = false;
        for (const auto &island_point : points) {
            const Point& p = island_point->point;
            Point abs_diff(fabs(p.x() - chck_point.x()),
                           fabs(p.y() - chck_point.y()));
            if (abs_diff.x() < min_distance && abs_diff.y() < min_distance) {
                double distance = sqrt((double) abs_diff.x() * abs_diff.x() +
                                       (double) abs_diff.y() * abs_diff.y());
                if (min_distance > distance) {
                    min_distance = distance;
                    exist_close_support_point = true;
                }
            }
        }
        if (!exist_close_support_point) is_island_supported = false;
    }

    bool is_all_points_inside_island = true;
    for (const auto &point : points)
        if (!island.contains(point->point))
            is_all_points_inside_island = false;
    
#ifdef STORE_ISLAND_ISSUES
    if (!is_island_supported || !is_all_points_inside_island) { // visualize
        static int  counter = 0;
        BoundingBox bb;
        for (const Point &pt : island.contour.points) bb.merge(pt);
        SVG svg(STORE_ISLAND_ISSUES + std::string("Error") + std::to_string(++counter) + ".svg", bb);
        svg.draw(island, "blue", 0.5f);
        for (auto& p : points)
            svg.draw(p->point, island.contains(p->point) ? "lightgreen" : "red", config.head_radius);
        for (size_t index = 0; index < chck_points.size(); ++index) {
            const Point &chck_point = chck_points[index];
            double       distance   = point_distances[index];
            bool         isOk       = distance < max_distance;
            std::string  color      = (isOk) ? "gray" : "red";
            svg.draw(chck_point, color, config.head_radius / 4);
        }
    }
#endif // STORE_ISLAND_ISSUES

    CHECK(!points.empty());
    CHECK(is_all_points_inside_island);
    // CHECK(is_island_supported); // TODO: skip special cases with one point and 2 points

    return points;
}

SampleConfig create_sample_config(double size) {
    float head_diameter = .4f;
    return SampleConfigFactory::create(head_diameter);

    //coord_t max_distance = 3 * size + 0.1;
    //SampleConfig cfg;
    //cfg.head_radius = size / 4;
    //cfg.minimal_distance_from_outline = cfg.head_radius;
    //cfg.maximal_distance_from_outline = max_distance/4;
    //cfg.max_length_for_one_support_point = 2*size;
    //cfg.max_length_for_two_support_points = 4*size;
    //cfg.thin_max_width = size;
    //cfg.thick_min_width = cfg.thin_max_width;
    //cfg.thick_outline_max_distance       = max_distance;

    //cfg.minimal_move       = static_cast<coord_t>(size/30);
    //cfg.count_iteration = 100; 
    //cfg.max_align_distance = 0;
    //return cfg;
} 

#ifdef STORE_SAMPLE_INTO_SVG_FILES
namespace {
void store_sample(const SupportIslandPoints &samples, const ExPolygon &island) { 
    static int counter = 0;
    BoundingBox bb(island.contour.points);
    SVG svg((STORE_SAMPLE_INTO_SVG_FILES + std::to_string(counter++) + ".svg").c_str(), bb); 

    double mm = scale_(1);
    svg.draw(island, "lightgray");
    for (const auto &s : samples) 
        svg.draw(s->point, "blue", 0.2*mm);
    

    // draw resolution
    Point p(bb.min.x() + 1e6, bb.max.y() - 2e6);
    svg.draw_text(p, (std::to_string(samples.size()) + " samples").c_str(), "black");
    svg.draw_text(p - Point(0., 1.8e6), "Scale 1 cm ", "black");
    Point  start = p - Point(0., 2.3e6);
    svg.draw(Line(start + Point(0., 5e5), start + Point(10*mm, 5e5)), "black", 2e5);
    svg.draw(Line(start + Point(0., -5e5), start + Point(10*mm, -5e5)), "black", 2e5);
    svg.draw(Line(start + Point(10*mm, 5e5), start + Point(10*mm, -5e5)), "black", 2e5);
    for (int i=0; i<10;i+=2)
        svg.draw(Line(start + Point(i*mm, 0.), start + Point((i+1)*mm, 0.)), "black", 1e6);
}
} // namespace
#endif // STORE_SAMPLE_INTO_SVG_FILES

/// <summary>
/// Check for correct sampling of island
/// </summary>
TEST_CASE("Uniform sample test islands", "[SupGen], [VoronoiSkeleton]")
{
    //set_logging_level(5);
    float head_diameter = .4f;
    SampleConfig cfg = SampleConfigFactory::create(head_diameter);
    //cfg.path = "C:/data/temp/islands/<<order>>.svg";
    ExPolygons islands = createTestIslands(7 * scale_(head_diameter));
    for (ExPolygon &island : islands) {
        // information for debug which island cause problem
        [[maybe_unused]] size_t debug_index = &island - &islands.front(); 

        SupportIslandPoints points = test_island_sampling(island, cfg);
#ifdef STORE_SAMPLE_INTO_SVG_FILES
        store_sample(points, island);
#endif // STORE_SAMPLE_INTO_SVG_FILES
        
        double angle  = 3.14 / 3; // cca 60 degree

        island.rotate(angle);
        SupportIslandPoints pointsR = test_island_sampling(island, cfg);
#ifdef STORE_SAMPLE_INTO_SVG_FILES
        store_sample(pointsR, island);
#endif // STORE_SAMPLE_INTO_SVG_FILES

        // points count should be the same
        //CHECK(points.size() == pointsR.size())
    }
}

TEST_CASE("Sample island with config", "[SupportIsland]") {
    // set_logging_level(5);
    SampleConfig cfg{
        /*thin_max_distance*/ 5832568,
        /*thick_inner_max_distance*/ 7290710,
        /*thick_outline_max_distance*/ 5468032,
        /*head_radius*/ 250000,
        /*minimal_distance_from_outline*/ 250000,
        /*maximal_distance_from_outline*/ 1944189,
        /*max_length_for_one_support_point*/ 1869413,
        /*max_length_for_two_support_points*/ 7290710,
        /*max_length_ratio_for_two_support_points*/ 0.250000000f,
        /*thin_max_width*/ 4673532,
        /*thick_min_width*/ 4019237,
        /*min_part_length*/ 5832568,
        /*minimal_move*/ 100000,
        /*count_iteration*/ 30,
        /*max_align_distance*/ 3645355,
        /*simplification_tolerance*/ 50000.000000000007
        //*path*/, "C:/data/temp/islands/<<order>>.svg" // need define OPTION_TO_STORE_ISLAND in SampleConfig.hpp
    };
    std::string dir = std::string(TEST_DATA_DIR PATH_SEPARATOR) + "sla_islands/";
    ExPolygon island = load_svg(dir + "SPE-2709.svg"); // Bad field creation
    SupportIslandPoints points = test_island_sampling(island, cfg);
    // in time of write poins.size() == 39
    CHECK(points.size() > 22); // not only thin parts
}

TEST_CASE("Disable visualization", "[hide]") 
{
    CHECK(true);
#ifdef STORE_SAMPLE_INTO_SVG_FILES
    CHECK(false);
#endif // STORE_SAMPLE_INTO_SVG_FILES
#ifdef STORE_ISLAND_ISSUES
    CHECK(false);
#endif // STORE_ISLAND_ISSUES
#ifdef USE_ISLAND_GUI_FOR_SETTINGS
    CHECK(false);
#endif // USE_ISLAND_GUI_FOR_SETTINGS
    CHECK(is_uniform_support_island_visualization_disabled());
}

TEST_CASE("SPE-2714 3DBenchy - Sample island with config", "[SupportIsland]") {
    // set_logging_level(5);
    SampleConfig cfg{
        /*thin_max_distance*/ 5832568,
        /*thick_inner_max_distance*/ 7290710,
        /*thick_outline_max_distance*/ 5468032,
        /*head_radius*/ 250000,
        /*minimal_distance_from_outline*/ 250000,
        /*maximal_distance_from_outline*/ 1944189,
        /*max_length_for_one_support_point*/ 1869413,
        /*max_length_for_two_support_points*/ 7290710,
        /*max_length_ratio_for_two_support_points*/ 0.250000000f,
        /*thin_max_width*/ 4673532,
        /*thick_min_width*/ 4019237,
        /*min_part_length*/ 5832568,
        /*minimal_move*/ 100000,
        /*count_iteration*/ 30,
        /*max_align_distance*/ 3645355,
        /*simplification_tolerance*/ 50000.000000000007
        //*path*/, "C:/data/temp/islands/spe_2714_<<order>>.svg" // define OPTION_TO_STORE_ISLAND in SampleConfig.hpp
    };
    std::string dir = std::string(TEST_DATA_DIR PATH_SEPARATOR) + "sla_islands/";
    ExPolygon island = load_svg(dir + "SPE-2714.svg"); // Bad field creation
    SupportIslandPoints points = test_island_sampling(island, cfg);
    CHECK(points.size() > 22); // Before fix it not finished
}

namespace {
// Thrown by legacy_verify() when its recursion gets too deep, so a grid point
// that strays into a non-converging input fails the test case instead of
// overflowing the test process's stack.
struct LegacyVerifyTooDeep : std::runtime_error
{
    LegacyVerifyTooDeep() : std::runtime_error("legacy_verify recursion deeper than 50") {}
};

// SampleConfigFactory::verify() as of a114d13bd: the reference the bit-for-bit
// checks compare against. Two changes: the three asserts are removed (they
// fire whenever a check clamps, i.e. above ~205% density), and the recursion
// carries a depth guard.
bool legacy_verify(SampleConfig &cfg, int depth = 0) {
    if (depth > 50)
        throw LegacyVerifyTooDeep();
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
    if (!res) while (!legacy_verify(cfg, depth + 1));
    return res;
}

// SampleConfigFactory::create() as of a114d13bd without its closing verify():
// the raw defaults, before anything has clamped them. The leading assert is
// dropped along with the asserts in legacy_verify().
SampleConfig legacy_create_unverified(float support_head_diameter_in_mm) {
    SampleConfig result;
    result.head_radius = static_cast<coord_t>(scale_(support_head_diameter_in_mm/2));

    double head_area = M_PI * sqr(support_head_diameter_in_mm / 2); // Pi r^2
    result.max_length_for_one_support_point = static_cast<coord_t>(scale_(head_area * 2.9 + 1.3));
    result.max_length_for_two_support_points =
        static_cast<coord_t>(result.max_length_for_one_support_point * 3.9);
    result.thin_max_width = static_cast<coord_t>(result.max_length_for_one_support_point * 2.5);
    result.thick_min_width = static_cast<coord_t>(result.max_length_for_one_support_point * 2.15);
    result.thin_max_distance = static_cast<coord_t>(result.max_length_for_two_support_points * 0.8);
    result.thick_inner_max_distance = result.max_length_for_two_support_points; // 6.5mm
    result.thick_outline_max_distance = static_cast<coord_t>(result.max_length_for_two_support_points * 0.75); // 4.875mm
    result.minimal_distance_from_outline = result.head_radius;           // 0.2mm
    result.maximal_distance_from_outline = result.thin_max_distance / 3; // 1.73mm
    result.min_part_length = result.thin_max_distance;                   // 5.2mm
    result.minimal_move = scale_(0.1); // 0.1 mm is enough
    result.count_iteration = 30; // speed VS precission
    result.max_align_distance = result.max_length_for_two_support_points / 2;
    return result;
}

// SampleConfigFactory::create() as of a114d13bd, calling legacy_verify().
SampleConfig legacy_create(float support_head_diameter_in_mm) {
    SampleConfig result = legacy_create_unverified(support_head_diameter_in_mm);
    legacy_verify(result);
    return result;
}

// SampleConfigFactory::apply_density() as of a114d13bd, calling legacy_verify().
SampleConfig legacy_apply_density(const SampleConfig &current, float density) {
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
    legacy_verify(result);
    return result;
}

// SampleConfigFactory::apply_min_distance() as of a114d13bd, calling legacy_verify().
SampleConfig legacy_apply_min_distance(const SampleConfig &current, float min_distance_mm) {
    if (min_distance_mm <= 0.f)
        return current;

    const coord_t floor_distance = static_cast<coord_t>(scale_(min_distance_mm));
    SampleConfig result = current; // copy
    result.thin_max_distance = std::max(result.thin_max_distance, floor_distance);
    result.thick_inner_max_distance = std::max(result.thick_inner_max_distance, floor_distance);
    result.thick_outline_max_distance = std::max(result.thick_outline_max_distance, floor_distance);
    legacy_verify(result);
    return result;
}

// Density as SLAPrintSteps.cpp derives it from support_points_density_relative.
float density_from_percent(int percent) { return float(percent / 100.f); }

// thin_max_distance as legacy_apply_density() scales it, before verify().
// Below minimal_distance_from_outline the legacy code never converges.
coord_t legacy_scaled_thin_max_distance(const SampleConfig &cfg, float density) {
    if (is_approx(density, 1.f))
        return cfg.thin_max_distance;
    if (density < .1f)
        density = .1f;
    return static_cast<coord_t>(cfg.thin_max_distance / density);
}

// The chain SLAPrintSteps.cpp runs for each object.
SampleConfig derive_island_config(float head_diameter, int percent, float min_distance_mm) {
    return SampleConfigFactory::apply_min_distance(
        SampleConfigFactory::apply_density(
            SampleConfigFactory::create(head_diameter), density_from_percent(percent)),
        min_distance_mm);
}

void check_same_scalars(const SampleConfig &expected, const SampleConfig &actual) {
    CHECK(actual.thin_max_distance == expected.thin_max_distance);
    CHECK(actual.thick_inner_max_distance == expected.thick_inner_max_distance);
    CHECK(actual.thick_outline_max_distance == expected.thick_outline_max_distance);
    CHECK(actual.head_radius == expected.head_radius);
    CHECK(actual.minimal_distance_from_outline == expected.minimal_distance_from_outline);
    CHECK(actual.maximal_distance_from_outline == expected.maximal_distance_from_outline);
    CHECK(actual.max_length_for_one_support_point == expected.max_length_for_one_support_point);
    CHECK(actual.max_length_for_two_support_points == expected.max_length_for_two_support_points);
    CHECK(actual.max_length_ratio_for_two_support_points == expected.max_length_ratio_for_two_support_points);
    CHECK(actual.thin_max_width == expected.thin_max_width);
    CHECK(actual.thick_min_width == expected.thick_min_width);
    CHECK(actual.min_part_length == expected.min_part_length);
    CHECK(actual.minimal_move == expected.minimal_move);
    CHECK(actual.count_iteration == expected.count_iteration);
    CHECK(actual.max_align_distance == expected.max_align_distance);
    CHECK(actual.simplification_tolerance == expected.simplification_tolerance);
}

// The seven conditions verify() settles on, checked directly so a test does
// not rely on verify() to judge its own output. Computed in 64 bits so the
// check itself cannot overflow.
void check_constraints_hold(const SampleConfig &cfg) {
    const int64_t t = cfg.thin_max_distance;
    const int64_t r = cfg.head_radius;
    const int64_t m = cfg.minimal_distance_from_outline;
    CHECK(cfg.max_length_for_one_support_point < cfg.max_length_for_two_support_points);
    CHECK(cfg.thick_min_width < cfg.thin_max_width);
    CHECK(cfg.max_length_for_one_support_point <= 2 * t + 2 * r + 2 * m);
    CHECK(cfg.max_length_for_one_support_point >= 2 * r + 2 * m);
    CHECK(cfg.max_length_for_two_support_points <= 2 * t + 4 * r + 2 * m);
    CHECK(cfg.thin_max_width >= 2 * r + 2 * m);
    CHECK(cfg.thin_max_width <= 2 * t + 2 * r);
}

// Head diameters [mm] for the bit-for-bit grid; all at least 0.2 mm, where
// unsaturated results must match the legacy code. At 0.2 mm head_radius equals
// minimal_move, the edge where m_thresh stops being minimal_distance_from_outline.
const float grid_heads[] = {.2f, .3f, .4f, .5f, .6f, .8f, 1.f, 1.5f};

// Every whole percent from 10% up to the last one the legacy code survives,
// comparing the whole legacy chain (create included) with the current one.
void check_grid_matches_legacy(float min_distance_mm) {
    for (float head : grid_heads) {
        CAPTURE(head);
        const SampleConfig legacy_base = legacy_create(head);
        const SampleConfig base        = SampleConfigFactory::create(head);
        check_same_scalars(legacy_base, base);
        int last_percent = 0;
        for (int percent = 10;; ++percent) {
            const float density = density_from_percent(percent);
            if (legacy_scaled_thin_max_distance(legacy_base, density) < legacy_base.minimal_distance_from_outline)
                break;
            CAPTURE(percent);
            SampleConfig expected = legacy_apply_density(legacy_base, density);
            SampleConfig actual   = SampleConfigFactory::apply_density(base, density);
            if (min_distance_mm > 0.f) {
                expected = legacy_apply_min_distance(expected, min_distance_mm);
                actual   = SampleConfigFactory::apply_min_distance(actual, min_distance_mm);
            }
            check_same_scalars(expected, actual);
            last_percent = percent;
        }
        // Every head size in the grid survives past 2000%, so a lower stop means
        // the grid broke off early and compared far less than it claims.
        CHECK(last_percent > 2000);
    }
}
} // namespace

TEST_CASE("SampleConfig: density gives the legacy result bit for bit", "[SupportIsland][SampleConfig]") {
    check_grid_matches_legacy(0.f);
}

TEST_CASE("SampleConfig: density with min distance gives the legacy result bit for bit", "[SupportIsland][SampleConfig]") {
    check_grid_matches_legacy(1.5f);
}

TEST_CASE("SampleConfig: legacy_verify fails instead of overflowing the stack", "[SupportIsland][SampleConfig]") {
    SampleConfig cfg = SampleConfigFactory::create(.6f);
    cfg.thin_max_distance = cfg.minimal_distance_from_outline / 2; // no solution
    CHECK_THROWS_AS(legacy_verify(cfg), LegacyVerifyTooDeep);
}

TEST_CASE("SampleConfig: thin spacing saturates past the crash threshold", "[SupportIsland][SampleConfig]") {
    // 2205% is the first whole percent that overflowed the stack with a 0.6 mm head.
    const coord_t min_distance = static_cast<coord_t>(scale_(1.5f));
    coord_t previous_inner = std::numeric_limits<coord_t>::max();
    for (int percent : {2205, 5000, 10000}) {
        CAPTURE(percent);
        SampleConfig cfg;
        REQUIRE_NOTHROW(cfg = derive_island_config(.6f, percent, 0.f));
        CHECK(cfg.thin_max_distance == cfg.minimal_distance_from_outline);
        CHECK(cfg.thick_inner_max_distance < previous_inner);
        check_constraints_hold(cfg);
        previous_inner = cfg.thick_inner_max_distance;

        // A min distance above the saturated spacing lifts all three to it.
        SampleConfig raised;
        REQUIRE_NOTHROW(raised = derive_island_config(.6f, percent, 1.5f));
        CHECK(raised.thin_max_distance == min_distance);
        CHECK(raised.thick_inner_max_distance >= min_distance);
        CHECK(raised.thick_outline_max_distance >= min_distance);
        check_constraints_hold(raised);
    }
}

TEST_CASE("SampleConfig: thick spacings keep the legacy formulas past saturation", "[SupportIsland][SampleConfig]") {
    const SampleConfig base = SampleConfigFactory::create(.6f);
    for (const auto &[percent, inner_mm] : std::vector<std::pair<int, double>>{{3100, 1.48}, {4200, 1.27}}) {
        CAPTURE(percent);
        const float density = density_from_percent(percent);
        const SampleConfig cfg = SampleConfigFactory::apply_density(base, density);
        REQUIRE(cfg.thin_max_distance == cfg.minimal_distance_from_outline); // saturated
        // Square-root scaling, as the unsaturated path uses - not linear.
        CHECK(cfg.thick_inner_max_distance == static_cast<coord_t>(
            std::sqrt(sqr((double) base.thick_inner_max_distance) / density)));
        CHECK(std::abs(unscale<double>(cfg.thick_inner_max_distance) - inner_mm) <= .01);
        // Still above its floor here, so the linear formula shows through.
        CHECK(cfg.thick_outline_max_distance == static_cast<coord_t>(
            base.thick_outline_max_distance / density));
    }
}

TEST_CASE("SampleConfig: extreme densities return with every spacing floored", "[SupportIsland][SampleConfig]") {
    for (float head : {0.f, .1f, .6f}) {
        for (int percent : {5000, 10000, INT_MAX}) {
            CAPTURE(head, percent);
            SampleConfig cfg;
            REQUIRE_NOTHROW(cfg = derive_island_config(head, percent, 0.f));
            CHECK(cfg.thin_max_distance >= std::max(cfg.minimal_distance_from_outline, cfg.minimal_move));
            CHECK(cfg.thick_outline_max_distance >= cfg.minimal_move);
            CHECK(cfg.thick_inner_max_distance >= std::max(2 * cfg.head_radius, cfg.minimal_move));
            check_constraints_hold(cfg);
        }
    }

    const SampleConfig head_06 = derive_island_config(.6f, INT_MAX, 0.f);
    CHECK(head_06.thick_outline_max_distance == head_06.minimal_move);
    CHECK(head_06.thick_inner_max_distance == 2 * head_06.head_radius);

    const SampleConfig head_0 = derive_island_config(0.f, INT_MAX, 0.f);
    CHECK(head_0.minimal_distance_from_outline == 0);
    CHECK(head_0.thin_max_distance >= head_0.minimal_move);
    CHECK(head_0.thick_outline_max_distance >= head_0.minimal_move);
    CHECK(head_0.thick_inner_max_distance >= head_0.minimal_move);

    REQUIRE_NOTHROW(derive_island_config(0.f, 100, 0.f));
}

TEST_CASE("SampleConfig: floors only apply once thin spacing saturates", "[SupportIsland][SampleConfig]") {
    SECTION("0.2 mm head at 4100%: outline already under minimal_move, thin not saturated") {
        const SampleConfig legacy_base = legacy_create(.2f);
        const SampleConfig base        = SampleConfigFactory::create(.2f);
        const float density = density_from_percent(4100);
        REQUIRE(legacy_scaled_thin_max_distance(legacy_base, density) >=
                std::max(legacy_base.minimal_distance_from_outline, legacy_base.minimal_move));
        REQUIRE(static_cast<coord_t>(legacy_base.thick_outline_max_distance / density) < legacy_base.minimal_move);
        const SampleConfig cfg = SampleConfigFactory::apply_density(base, density);
        check_same_scalars(legacy_apply_density(legacy_base, density), cfg);
        CHECK(cfg.thick_outline_max_distance < cfg.minimal_move);
    }
    SECTION("0.1 mm head: unchanged above minimal_move, saturated at it below") {
        const SampleConfig legacy_base = legacy_create(.1f);
        const SampleConfig base        = SampleConfigFactory::create(.1f);

        const float unsaturated = density_from_percent(4000);
        REQUIRE(legacy_scaled_thin_max_distance(legacy_base, unsaturated) >= legacy_base.minimal_move);
        check_same_scalars(legacy_apply_density(legacy_base, unsaturated),
                           SampleConfigFactory::apply_density(base, unsaturated));

        // The legacy code still converges here (thin >= minimal_distance_from_outline),
        // but the result is held at minimal_move by design.
        const float saturated = density_from_percent(6000);
        const coord_t thin = legacy_scaled_thin_max_distance(legacy_base, saturated);
        REQUIRE(thin >= legacy_base.minimal_distance_from_outline);
        REQUIRE(thin < legacy_base.minimal_move);
        const SampleConfig cfg = SampleConfigFactory::apply_density(base, saturated);
        CHECK(cfg.thin_max_distance == cfg.minimal_move);
        CHECK(cfg.thick_outline_max_distance >= cfg.minimal_move);
    }
}

TEST_CASE("SampleConfig: verify() reports a contradictory config with the engine error code", "[SupportIsland][SampleConfig]") {
    SampleConfig cfg = SampleConfigFactory::create(.6f);
    cfg.thin_max_distance = cfg.minimal_distance_from_outline / 2; // thin_max_width has no valid value
    bool thrown = false;
    try {
        SampleConfigFactory::verify(cfg);
    } catch (const Slic3r::RuntimeError &e) {
        thrown = true;
        const auto *carrier = dynamic_cast<const EngineErrorCarrier *>(&e);
        REQUIRE(carrier != nullptr);
        REQUIRE(carrier->engine_error().code.has_value());
        CHECK(*carrier->engine_error().code == EngineErrorCode::SUPPORT_POINT_SAMPLING_FAILED);
    }
    CHECK(thrown);
}

TEST_CASE("SampleConfig: create() defaults pass verify() on the first pass", "[SupportIsland][SampleConfig]") {
    // Every head size the tests pass to create(), so a default that starts
    // needing a clamp fails here rather than tripping assert(clean) elsewhere.
    for (float head : {0.f, .1f, .2f, .3f, .4f, .5f, .6f, .8f, 1.f, 1.5f}) {
        CAPTURE(head);
        // The raw defaults, before any verify(): true means nothing needed a clamp.
        SampleConfig raw = legacy_create_unverified(head);
        CHECK(SampleConfigFactory::verify(raw));
        check_same_scalars(raw, SampleConfigFactory::create(head));
    }
}

TEST_CASE("SampleConfig: a density that makes verify() clamp does not trip an assert", "[SupportIsland][SampleConfig]") {
    // At 300% verify() has to lower max_length_for_two_support_points. An
    // assert on that path only fires in Debug; Release and RelWithDebInfo
    // (the builds this runs in) still check that the clamp happens and nothing
    // throws.
    const SampleConfig base = SampleConfigFactory::create(.6f);
    SampleConfig cfg;
    REQUIRE_NOTHROW(cfg = SampleConfigFactory::apply_density(base, 3.f));
    CHECK(cfg.max_length_for_two_support_points < base.max_length_for_two_support_points);
}

TEST_CASE("SampleConfig: no head size or density makes the factory throw", "[SupportIsland][SampleConfig]") {
    // Everything below 10% clamps to the same density, so 0% stands for it.
    std::vector<int> percents{0};
    for (int percent = 10; percent <= 10000; ++percent)
        percents.push_back(percent);
    for (int percent = 10000 + 997; percent < 1000000; percent += 997)
        percents.push_back(percent);
    percents.push_back(INT_MAX);

    for (float head : {0.f, .1f, .2f, .3f, .4f, .5f, .6f, .8f, 1.f, 1.5f}) {
        const SampleConfig base = SampleConfigFactory::create(head);
        for (int percent : percents) {
            CAPTURE(head, percent);
            SampleConfig scaled;
            REQUIRE_NOTHROW(scaled = SampleConfigFactory::apply_density(base, density_from_percent(percent)));
            // 0.15 mm can land between a saturated thin spacing and the thick
            // ones, so only some spacings get raised; 1.5 mm raises them all.
            for (float min_distance : {.15f, 1.5f}) {
                CAPTURE(min_distance);
                REQUIRE_NOTHROW(SampleConfigFactory::apply_min_distance(scaled, min_distance));
            }
        }
    }
}
