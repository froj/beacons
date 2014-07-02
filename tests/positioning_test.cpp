#include <cmath>
#include <assert.h>

#include <iostream>

#include "CppUTest/TestHarness.h"
#include "CppUTest/SimpleString.h"

extern "C" {
#include "../src/positioning.h"
}

struct Angles;
struct Triangle;

class Vec2D
{
    private:
        float _x;
        float _y;
    public:
        Vec2D(float x, float y) : _x(x), _y(y) {}
        float dot(const Vec2D & other) const;
        float length() const;
        float directed_angle_to(const Vec2D & other) const;
        Angles angles_relative_to_triangle(const Triangle & t) const;
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

struct Triangle
{
    Vec2D a;
    Vec2D b;
    Vec2D c;
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

Angles Vec2D::angles_relative_to_triangle(const Triangle & t) const
{
    Vec2D PA = t.a - *this;
    Vec2D PB = t.b - *this;
    Vec2D PC = t.c - *this;

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

static Vec2D angles_to_coord(const Angles & angles)
{
    position_t pos = positioning_position_from_angles(angles.alpha, angles.beta, angles.gamma);

    return Vec2D(pos.x, pos.y);
}

static float random(float lower, float upper)
{
    float length = upper - lower;
    float r = ((float) std::rand() / RAND_MAX);

    return r*length + lower;
}

static Vec2D random_point_on_table()
{
    return Vec2D(random(0.0f,3.0f), random(0.0f,2.0f));
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

static Vec2D point_a = Vec2D(POINT_A_X, POINT_A_Y);
static Vec2D point_b = Vec2D(POINT_B_X, POINT_B_Y);
static Vec2D point_c = Vec2D(POINT_C_X, POINT_C_Y);

static uint32_t rng_seed;

static Triangle reference_triangle = {
    point_a,
    point_b,
    point_c
};

TEST_GROUP(PositioningTestGroup)
{
    void setup(void)
    {
        rng_seed = time(NULL);
        std::srand(rng_seed);

        std::cout << "Random seed used in test: " << rng_seed << std::endl;
    }

    void teardown(void)
    {
    }
};

TEST(PositioningTestGroup, FixPoint)
{
    Vec2D p = Vec2D(1.0f,1.0f);
    Vec2D q = angles_to_coord(p.angles_relative_to_triangle(reference_triangle));
    CHECK_EQUAL(p,q);
}

TEST(PositioningTestGroup, RandomizedPoints)
{
    static const int max_tests = 100;
    for(int i = 0; i < max_tests; i++)
    {
        Vec2D random_p = random_point_on_table();
        Vec2D after_transformation = angles_to_coord(random_p.angles_relative_to_triangle(reference_triangle));
        CHECK_EQUAL(random_p, after_transformation);
    }
};
