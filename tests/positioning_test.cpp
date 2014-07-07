#include <cmath>
#include <assert.h>

#include <iostream>

#include "CppUTest/TestHarness.h"
#include "CppUTest/SimpleString.h"

extern "C" {
#include "../src/positioning.h"
}

#define POINT_A_X (3.0f)
#define POINT_A_Y (1.0f)

#define POINT_B_X (0.0f)
#define POINT_B_Y (2.0f)

#define POINT_C_X (0.0f)
#define POINT_C_Y (0.0f)

struct Angles;

class Vec2D
{
    private:
        float _x;
        float _y;
    public:
        Vec2D(float x, float y) : _x(x), _y(y) {}
        Vec2D(const position_t * pos) : _x(pos->x), _y(pos->y) {}
        float dot(const Vec2D & other) const;
        float length() const;
        float directed_angle_to(const Vec2D & other) const;
        Angles angles_relative_to_triangle(const reference_triangle_t * t) const;
        Vec2D operator-(const Vec2D & other) const;
        bool operator==(const Vec2D & other) const;
        bool operator!=(const Vec2D & other) const;
        float get_x() const;
        float get_y() const;
};

struct Angles
{
    float alpha;
    float beta;
    float gamma;
};

float Vec2D::get_x() const
{
    return _x;
}

float Vec2D::get_y() const
{
    return _y;
}

bool Vec2D::operator==(const Vec2D & other) const
{
    static const float EPSILON = 0.0001;
    float dx = std::fabs(_x - other._x);
    float dy = std::fabs(_y - other._y);

    return dx < EPSILON && dy < EPSILON;
}

bool Vec2D::operator!=(const Vec2D & other) const
{
    return !(*this == other);
}

float Vec2D::dot(const Vec2D & other) const
{
    return _x*other._x + _y*other._y;
}

float Vec2D::length() const
{
    return sqrt(_x*_x + _y*_y);
}

// see: http://stackoverflow.com/questions/21483999/using-atan2-to-find-angle-between-two-vectors
float Vec2D::directed_angle_to(const Vec2D & other) const
{
    float angle = std::atan2(other._y, other._x) - std::atan2(_y, _x);

    return angle < 0 ? angle + 2 * M_PI : angle;
}

Vec2D Vec2D::operator-(const Vec2D & other) const
{
    return Vec2D(this->_x - other._x, this->_y - other._y);
}

Angles Vec2D::angles_relative_to_triangle(const reference_triangle_t * t) const
{
    Vec2D PA = Vec2D(t->point_a) - *this;
    Vec2D PB = Vec2D(t->point_b) - *this;
    Vec2D PC = Vec2D(t->point_c) - *this;

    float alpha_ = PB.directed_angle_to(PC);
    float beta_ = PC.directed_angle_to(PA);
    float gamma_ = PA.directed_angle_to(PB);

    Angles result;
    result.alpha = alpha_;
    result.beta = beta_;
    result.gamma = gamma_;

    assert(std::fabs(alpha_ + beta_ + gamma_ - 2 * M_PI) < 0.0001);

    return result;
}

static Vec2D angles_to_coord(const Angles & angles, const reference_triangle_t * t)
{
    position_t pos = {0, 0};
    positioning_from_angles(angles.alpha, angles.beta, angles.gamma, t, &pos);

    return Vec2D(&pos);
}

static SimpleString StringFrom(const Vec2D & v)
{
    SimpleString s = SimpleString();
    s += SimpleString("(");
    s += StringFrom(v.get_x());
    s += SimpleString(",");
    s += StringFrom(v.get_y());
    s += SimpleString(")");

    return s;
}

static position_t point_a = {POINT_A_X, POINT_A_Y};
static position_t point_b = {POINT_B_X, POINT_B_Y};
static position_t point_c = {POINT_C_X, POINT_C_Y};

static reference_triangle_t reference_triangle = {NULL, NULL, NULL, 0, 0, 0};

TEST_GROUP(ReferenceTriangleTestGroup)
{
    void setup(void)
    {

    }

    void teardown(void)
    {

    }
};

TEST(ReferenceTriangleTestGroup, FirstArgumentNull)
{
    position_t p1 = {0, 0};
    position_t p2 = {1, 0};
    position_t p3 = {0, 1};

    reference_triangle_t t = {NULL, NULL, NULL, 0, 0, 0};

    positioning_reference_triangle_from_points(NULL, &p2, &p3, &t);
    POINTERS_EQUAL(NULL, t.point_a);
    POINTERS_EQUAL(NULL, t.point_b);
    POINTERS_EQUAL(NULL, t.point_c);
    DOUBLES_EQUAL(0.0, t.cotangent_at_a, 0.0001);
    DOUBLES_EQUAL(0.0, t.cotangent_at_b, 0.0001);
    DOUBLES_EQUAL(0.0, t.cotangent_at_c, 0.0001);
}

TEST(ReferenceTriangleTestGroup, SecondArgumentNull)
{
    position_t p1 = {0, 0};
    position_t p2 = {1, 0};
    position_t p3 = {0, 1};

    reference_triangle_t t = {NULL, NULL, NULL, 0, 0, 0};

    positioning_reference_triangle_from_points(&p1, NULL, &p3, &t);
    POINTERS_EQUAL(NULL, t.point_a);
    POINTERS_EQUAL(NULL, t.point_b);
    POINTERS_EQUAL(NULL, t.point_c);
    DOUBLES_EQUAL(0.0, t.cotangent_at_a, 0.0001);
    DOUBLES_EQUAL(0.0, t.cotangent_at_b, 0.0001);
    DOUBLES_EQUAL(0.0, t.cotangent_at_c, 0.0001);
}

TEST(ReferenceTriangleTestGroup, ThirdArgumentNull)
{
    position_t p1 = {0, 0};
    position_t p2 = {1, 0};
    position_t p3 = {0, 1};

    reference_triangle_t t = {NULL, NULL, NULL, 0, 0, 0};

    positioning_reference_triangle_from_points(&p1, &p2, NULL, &t);
    POINTERS_EQUAL(NULL, t.point_a);
    POINTERS_EQUAL(NULL, t.point_b);
    POINTERS_EQUAL(NULL, t.point_c);
    DOUBLES_EQUAL(0.0, t.cotangent_at_a, 0.0001);
    DOUBLES_EQUAL(0.0, t.cotangent_at_b, 0.0001);
    DOUBLES_EQUAL(0.0, t.cotangent_at_c, 0.0001);
}

TEST(ReferenceTriangleTestGroup, FourthArgumentNull)
{
    position_t p1 = {0, 0};
    position_t p2 = {1, 0};
    position_t p3 = {0, 1};

    reference_triangle_t t = {NULL, NULL, NULL, 0, 0, 0};

    positioning_reference_triangle_from_points(&p1, &p2, &p3, NULL);
    POINTERS_EQUAL(NULL, t.point_a);
    POINTERS_EQUAL(NULL, t.point_b);
    POINTERS_EQUAL(NULL, t.point_c);
    DOUBLES_EQUAL(0.0, t.cotangent_at_a, 0.0001);
    DOUBLES_EQUAL(0.0, t.cotangent_at_b, 0.0001);
    DOUBLES_EQUAL(0.0, t.cotangent_at_c, 0.0001);
}

TEST(ReferenceTriangleTestGroup, PointsWrongOrientation)
{
    position_t p1 = {0, 0};
    position_t p2 = {1, 0};
    position_t p3 = {0, 1};

    reference_triangle_t t = {NULL, NULL, NULL, 0, 0, 0};

    positioning_reference_triangle_from_points(&p3, &p2, &p1, &t);
    POINTERS_EQUAL(NULL, t.point_a);
    POINTERS_EQUAL(NULL, t.point_b);
    POINTERS_EQUAL(NULL, t.point_c);
    DOUBLES_EQUAL(0.0, t.cotangent_at_a, 0.0001);
    DOUBLES_EQUAL(0.0, t.cotangent_at_b, 0.0001);
    DOUBLES_EQUAL(0.0, t.cotangent_at_c, 0.0001);
}

TEST(ReferenceTriangleTestGroup, PointsColinear)
{
    position_t p1 = {0, 0};
    position_t p2 = {1, 0};
    position_t p3 = {2, 0};

    reference_triangle_t t = {NULL, NULL, NULL, 0, 0, 0};

    positioning_reference_triangle_from_points(&p1, &p2, &p3, &t);
    POINTERS_EQUAL(NULL, t.point_a);
    POINTERS_EQUAL(NULL, t.point_b);
    POINTERS_EQUAL(NULL, t.point_c);
    DOUBLES_EQUAL(0.0, t.cotangent_at_a, 0.0001);
    DOUBLES_EQUAL(0.0, t.cotangent_at_b, 0.0001);
    DOUBLES_EQUAL(0.0, t.cotangent_at_c, 0.0001);
}

TEST(ReferenceTriangleTestGroup, NormalOperation)
{
    position_t p1 = {0, 0};
    position_t p2 = {1, 0};
    position_t p3 = {0, 1};

    reference_triangle_t t = {NULL, NULL, NULL, 0, 0, 0};

    positioning_reference_triangle_from_points(&p1, &p2, &p3, &t);
    POINTERS_EQUAL(&p1, t.point_a);
    POINTERS_EQUAL(&p2, t.point_b);
    POINTERS_EQUAL(&p3, t.point_c);
    DOUBLES_EQUAL(0.0, t.cotangent_at_a, 0.0001);
    DOUBLES_EQUAL(1.0, t.cotangent_at_b, 0.0001);
    DOUBLES_EQUAL(1.0, t.cotangent_at_c, 0.0001);
}

TEST_GROUP(PositioningTestGroup)
{
    void setup(void)
    {
        positioning_reference_triangle_from_points(&point_a, &point_b, &point_c, &reference_triangle);
    }

    void teardown(void)
    {
    }
};

TEST(PositioningTestGroup, FixPoint)
{
    Vec2D p = Vec2D(1.0f,1.0f);
    Vec2D q = angles_to_coord(p.angles_relative_to_triangle(&reference_triangle), &reference_triangle);
    CHECK_EQUAL(p,q);
}

TEST(PositioningTestGroup, PointA)
{
   Vec2D p = Vec2D(&point_a);
   Vec2D res = angles_to_coord(p.angles_relative_to_triangle(&reference_triangle), &reference_triangle);
   CHECK_EQUAL(p, res);
}

TEST(PositioningTestGroup, PointB)
{
   Vec2D p = Vec2D(&point_b);
   Vec2D res = angles_to_coord(p.angles_relative_to_triangle(&reference_triangle), &reference_triangle);
   CHECK_EQUAL(p, res);
}

TEST(PositioningTestGroup, PointC)
{
   Vec2D p = Vec2D(&point_c);
   Vec2D res = angles_to_coord(p.angles_relative_to_triangle(&reference_triangle), &reference_triangle);
   CHECK_EQUAL(p, res);
}

