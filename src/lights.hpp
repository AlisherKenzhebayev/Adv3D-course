#pragma once

#include <vector>
#include <cmath>
#include <utility>
#include <tuple>
#include <stdexcept>
#include "math.hpp"
#include "rng.hpp"

class AbstractLight
{
public:

    /**
     * Randomly chooses a point on the light source
     * Arguments:
     *  - origin = our current position in the scene
     *  - rng = random generator
     * Returns:
     *  - a randomly sampled point on the light source
     *  - the illumination intensity corresponding to the sampled direction
     *  - the probability density (PDF) of choosing this point
     */
    virtual std::tuple<Vec3f, Vec3f, float> SamplePointOnLight(const Vec3f& origin, Rng& rng) const {
        throw std::logic_error("Not implemented");
    }

    /**
     * Returns the probability density corresponding to samplePointOnLight,
     * i.e., what is the probability that calling samplePointOnLight would randomly choose the given lightPoint
     * Arguments:
     *  - origin = our current position in the scene
     *  - lightPoint = the randomly sampled point on the light source
     */
    virtual float PDF(const Vec3f& origin, const Vec3f& lightPoint) const {
        throw std::logic_error("Not implemented");
    }

    /**
     * Returns the illumination intensity in the given direction
     * Arguments:
     *  - direction = direction towards the light source
     */
    virtual Vec3f Evaluate(const Vec3f& direction) const {
        throw std::logic_error("Not implemented");
    }

    virtual ~AbstractLight() = default;
};

//////////////////////////////////////////////////////////////////////////
class AreaLight : public AbstractLight
{
public:

    AreaLight(
        const Vec3f& aP0,
        const Vec3f& aP1,
        const Vec3f& aP2)
    {
        p0 = aP0;
        e1 = aP1 - aP0;
        e2 = aP2 - aP0;

        Vec3f normal = Cross(e1, e2);
        float len = normal.Length();
        mInvArea = 2.f / len; // Area of the triangle, inversed
        mFrame.SetFromZ(normal);
    }

    virtual std::tuple<Vec3f, Vec3f, float> SamplePointOnLight(const Vec3f& origin, Rng& rng) const override
    {
        Vec3f samplePoint;
        samplePoint = PointOnTriangle(p0, e1, e2, rng);
        Vec3f outgoingDirection = samplePoint - origin;
        float distanceSquared = outgoingDirection.LenSqr();
        float cos = (Dot(Normalize(Cross(e2, e1)), Normalize(outgoingDirection)));

        if (cos >= 0) {
            return { samplePoint, mRadiance, (mInvArea * distanceSquared) / cos };
        }
        else {
            return { samplePoint, 0.f , (mInvArea * distanceSquared) / cos };
        }
    }

    virtual float PDF(const Vec3f& origin, const Vec3f& lightPoint) const
    {
        Vec3f outgoingDirection = lightPoint - origin;
        float distanceSquared = outgoingDirection.LenSqr();
        float cos = (Dot(Normalize(Cross(e2, e1)), Normalize(outgoingDirection)));

        return (mInvArea * distanceSquared) / cos;
    }

    virtual Vec3f Evaluate(const Vec3f& direction) const override
    {
        return mRadiance;
    }

public:
    Vec3f p0, e1, e2;
    CoordinateFrame mFrame;
    Vec3f mRadiance;
    float mInvArea;
};

//////////////////////////////////////////////////////////////////////////
class PointLight : public AbstractLight
{
public:

    PointLight(const Vec3f& aPosition)
    {
        mPosition = aPosition;
    }

    virtual std::tuple<Vec3f, Vec3f, float> SamplePointOnLight(const Vec3f& origin, Rng& rng) const override {
        Vec3f outgoingDirection = mPosition - origin;
        float distanceSquared = outgoingDirection.LenSqr();

        return { mPosition, mIntensity / distanceSquared, 1.0f };
    }


    virtual float PDF(const Vec3f& origin, const Vec3f& lightPoint) const
    {
        return 0.f;
    }

    virtual Vec3f Evaluate(const Vec3f& direction) const override {
        return mIntensity;
    }

public:

    Vec3f mPosition;
    Vec3f mIntensity;
};


//////////////////////////////////////////////////////////////////////////
class BackgroundLight : public AbstractLight
{
public:
    BackgroundLight()
    {
        mBackgroundColor = Vec3f(135, 206, 250) / Vec3f(255.f);
        mRadius = 100.f; // a radius big enough to cover the whole scene
    }

    virtual std::tuple<Vec3f, Vec3f, float> SamplePointOnLight(const Vec3f& origin, Rng& rng) const override {
        Vec3f samplePoint;
        samplePoint = PointOnSphere(mRadius, rng);
        Vec3f outgoingDirection = samplePoint - origin;
        float distanceSquared = outgoingDirection.LenSqr();

        double Area = 4 * PI_F;
        float cos = (Dot(Normalize(samplePoint), Normalize(-outgoingDirection)));

        return { samplePoint, mBackgroundColor, 1.f / Area };
    }


    virtual float PDF(const Vec3f& origin, const Vec3f& lightPoint) const
    {
        double Area = 4 * PI_F;
        return 1.f / Area;
    }

    virtual Vec3f Evaluate(const Vec3f& direction) const override {
        return mBackgroundColor;
    }

public:

    Vec3f mBackgroundColor;
    float mRadius; // we model the background light as a huge sphere around the whole scene, with a given radius
};
