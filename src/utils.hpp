#pragma once

#include <vector>
#include <cmath>
#include <utility>
#include "math.hpp"
#include "rng.hpp"

#define EPSILON_COSINE 1e-6f
#define EPSILON_RAY    1e-3f

// sRGB luminance
float Luminance(const Vec3f& aRGB)
{
    return 0.212671f * aRGB.x +
        0.715160f * aRGB.y +
        0.072169f * aRGB.z;
}

// reflect vector through (0,0,1)
Vec3f ReflectLocal(const Vec3f& aVector)
{
    return Vec3f(-aVector.x, -aVector.y, aVector.z);
}

//////////////////////////////////////////////////////////////////////////
// Utilities for converting PDF between Area (A) and Solid angle (W)
// WtoA = PdfW * cosine / distance_squared
// AtoW = PdfA * distance_squared / cosine

float PdfWtoA(
    const float aPdfW,
    const float aDist,
    const float aCosThere)
{
    return aPdfW * std::abs(aCosThere) / Sqr(aDist);
}

float PdfAtoW(
    const float aPdfA,
    const float aDist,
    const float aCosThere)
{
    return aPdfA * Sqr(aDist) / std::abs(aCosThere);
}

/// <summary>
/// Given the triangle and rng, return a random point on that triangle
/// </summary>
Vec3f PointOnTriangle(Vec3f p0, Vec3f e1, Vec3f e2, Rng& rng) {
    float u = rng.GetFloat();
    float v = rng.GetFloat();
    Vec3f B = p0 + e1;
    Vec3f C = p0 + e2;

    float su0 = sqrt(u);
    return (1 - su0) * B + (v * su0) * C + (1 - v) * su0 * p0;
}

Vec3f PointOnSphere(float radius, Rng& rng) {
    Vec2f uv = rng.GetVec2f();

    float z = 1 - 2 * uv.x;
    float r = sqrt(std::max(0.f, 1.f - z * z));
    float phi = 2 * PI_F * uv.y;
    return radius * Vec3f(r * cos(phi), r * sin(phi), z);
}

Vec3f PointOnHemisphere(float radius, Rng& rng) {
    float u = rng.GetFloat();
    float v = rng.GetFloat();

    float r = sqrt(std::max(0.f, 1.f - u * u));
    float phi = 2 * PI_F * v;
    return radius * Vec3f(r * cos(phi), r * sin(phi), u);
}

Vec3f PointOnHemisphereCosineWeighted(float radius, Rng& rng) {
    float u = rng.GetFloat();
    float v = rng.GetFloat();

    float r = sqrt(std::max(0.f, 1.f - u));
    float phi = 2 * PI_F * v;
    return radius * Vec3f(r * cos(phi), r * sin(phi), sqrt(u));
}