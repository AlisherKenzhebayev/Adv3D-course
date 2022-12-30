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
/// Returns an ideal reflection vector, given the incident direction and a surface normal 
/// </summary>
/// <param name="incomingDirection"> - a normalized direction towards the previous (origin) point in the scene</param>
/// <param name="N"> - Surface normal </param>
/// <returns> Normalized ideal reflection </returns>
Vec3f IdealReflection(Vec3f incomingDirection, Vec3f N)
{
    return Normalize(2 * Dot(N, incomingDirection) * N - incomingDirection);
}

/// <summary>
/// Given the triangle and rng, return a random point on that triangle
/// </summary>
Vec3f PointOnTriangle(Vec3f p0, Vec3f e1, Vec3f e2, Rng& rng) 
{
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

Vec3f PointOnHemisphereCosWeightedSolid(float radius, Rng& rng) {
    float u = rng.GetFloat();
    float v = rng.GetFloat();

    float r = sqrt(std::max(0.f, 1.f - u));
    float phi = 2 * PI_F * v;
    return radius * Vec3f(r * cos(phi), r * sin(phi), sqrt(u));
}

Vec3f PointOnHemisphereCosLobeNormalPow(float radius, Rng& rng, float exp) {
    float u = rng.GetFloat();
    float v = rng.GetFloat();

    float p = 2.f / (exp + 1);
    float r = sqrt(std::max(0.f, 1.f - (float)pow(u, p)));
    float phi = 2 * PI_F * v;
    return radius * Vec3f(r * cos(phi), r * sin(phi), pow(u, p / 2.f));
} 

float getDiffusePdf(Vec3f sampledDirection)
{
    return Normalize(sampledDirection).z / PI_F;
}

float getSpecularPdf(Vec3f incomingDirection, Vec3f sampledDirection, float exp)
{
    Vec3f N = Vec3f(0.0f, 0.0f, 1.0f);
    Vec3f R = IdealReflection(incomingDirection, N);
    // Set the coordinate frame to a reflection vector.
    CoordinateFrame frame;
    frame.SetFromZ(R);
    // Convert back to a coordinate frame of a specular lobe
    Vec3f directionSpec = frame.ToLocal(sampledDirection);

    return (pow(Normalize(directionSpec).z, exp) * 0.5f * (exp + 1)) / PI_F;
}

float maxComponent(Vec3f v) {
    return std::max({ v.x, v.y, v.z });
}