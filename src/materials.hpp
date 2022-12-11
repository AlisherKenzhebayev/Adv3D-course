#pragma once 

#include <utility>
#include <tuple>
#include <stdexcept>
#include "math.hpp"
#include "rng.hpp"
#include "utils.hpp"

class Material
{
public:
    Material()
    {
        Reset();
    }

    void Reset()
    {
        mDiffuseReflectance = Vec3f(0);
        mPhongReflectance   = Vec3f(0);
        mPhongExponent      = 1.f;
    }

    /**
     * Randomly chooses an outgoing direction that is reflected from the material surface
     * Arguments:
     *  - incomingDirection = a normalized direction towards the previous (origin) point in the scene
     *  - rng = random generator
     * Returns:
     *  - a randomly sampled reflected outgoing direction (local) 
     *  - the intensity corresponding to the reflected light        * UNUSED
     *  - the probability density (PDF) of choosing this direction  * UNUSED
     */
    std::tuple<Vec3f, Vec3f, float> SampleReflectedDirection(const Vec3f& incomingDirection, Rng& rng) const {
        // Probe randomly for a direction, based on specularity and diffusedness
        float pDiffuse = std::max({ mDiffuseReflectance.x, mDiffuseReflectance.y, mDiffuseReflectance.z });
        float pSpecular = std::max({ mPhongReflectance.x, mPhongReflectance.y, mPhongReflectance.z });
        float normalization = 1.f / (pDiffuse + pSpecular);
        pDiffuse *= normalization;
        pSpecular *= normalization;

        // Generate a random direction, based on the random roll 
        float rNum = rng.GetFloat();
        Vec3f directionLocal = Vec3f(0.f);
        if (rNum <= pDiffuse)
        {
            // Do a diffuse, recalculate for specular 
            directionLocal = SampleDiffuseDirection(incomingDirection, rng);
        }
        else 
        {
            directionLocal = SampleSpecularDirection(incomingDirection, rng);
        }

        //Vec3f sampledDirection = PointOnHemisphereCosWeightedSolid(1.0f, rng);

        float Pdf = 0.f;//pDiffuse * getDiffusePdf(directionLocal) + pSpecular * getSpecularPdf(incomingDirection, directionLocal, mPhongExponent);
        Vec3f intensityReflected = Vec3f(0.f);

        return { directionLocal, intensityReflected, Pdf };
    }

    /**
     * Randomly chooses an outgoing direction based on cosine-weighted sampling
     * Arguments:
     *  - incomingDirection = a normalized direction towards the previous (origin) point in the scene
     *  - rng = random generator
     * Returns:
     *  - a randomly sampled reflected outgoing direction
     */
    Vec3f SampleDiffuseDirection(const Vec3f& incomingDirection, Rng& rng) const {
        Vec3f sampledDirection = PointOnHemisphereCosWeightedSolid(1.0f, rng);

        return sampledDirection;
    }

    /**
     * Randomly chooses an outgoing direction based on specular lobe + hemisphere sampling
     * Arguments:
     *  - incomingDirection = a normalized direction towards the previous (origin) point in the scene
     *  - rng = random generator
     * Returns:
     *  - a randomly sampled reflected outgoing direction (local space)
     */
    Vec3f SampleSpecularDirection(const Vec3f& incomingDirection, Rng& rng) const {
        // Get an ideal reflection vector
        Vec3f N = Vec3f(0.0f, 0.0f, 1.0f);
        Vec3f R = IdealReflection(incomingDirection, N);
        // Set the coordinate frame to a reflection vector.
        CoordinateFrame frame;
        frame.SetFromZ(R);

        Vec3f sampledDirection = PointOnHemisphereCosLobeNormalPow(1.0f, rng, mPhongExponent);

        return frame.ToWorld(sampledDirection);
    }

    /**
     * Returns the probability density corresponding to sampleReflectedDirection,
     * i.e., what is the probability that calling sampleReflectedDirection would randomly choose the given outgoingDirection
     * Arguments:
     *  - incomingDirection = a normalized direction towards the previous (origin) point in the scene
     *  - outgoingDirection = the randomly sampled (normalized) outgoing direction
     */
    float PDF(const Vec3f& incomingDirection, const Vec3f& outgoingDirection) const {
        float pDiffuse = std::max({ mDiffuseReflectance.x, mDiffuseReflectance.y, mDiffuseReflectance.z });
        float pSpecular = std::max({ mPhongReflectance.x, mPhongReflectance.y, mPhongReflectance.z });
        float normalization = 1.f / (pDiffuse + pSpecular);
        pDiffuse *= normalization;
        pSpecular *= normalization;
            
        return pDiffuse * getDiffusePdf(outgoingDirection) + pSpecular * getSpecularPdf(incomingDirection, outgoingDirection, mPhongExponent);
    }

    /**
     * Returns the intensity corresponding to the reflected light according to this material's BRDF
     * Arguments:
     *  - incomingDirection = a normalized direction towards the previous (origin) point in the scene
     *  - outgoingDirection = a normalized outgoing reflected direction
     */
    Vec3f EvaluateBRDF(const Vec3f& incomingDirection, const Vec3f& outgoingDirection) const {
        if (incomingDirection.z <= 0 && outgoingDirection.z <= 0) {
			return Vec3f(0);
        }

        /*if()
        printf("D: %f %f %f | S: %f %f %f", mDiffuseReflectance.x, mDiffuseReflectance.y, mDiffuseReflectance.z, 
            mPhongReflectance.x, mPhongReflectance.y, mPhongReflectance.z);*/

		Vec3f diffuseComponent = mDiffuseReflectance / PI_F;

        Vec3f N = Vec3f(0.0f, 0.0f, 1.0f);
        Vec3f R = IdealReflection(incomingDirection, N);
        float dotPow = pow(Dot(outgoingDirection, R), mPhongExponent);
        Vec3f glossyComponent = mPhongReflectance * dotPow * (mPhongExponent + 2) / (2 * PI_F);

		Vec3f outIntensity = (diffuseComponent + glossyComponent);
        return (outIntensity);
    }

    Vec3f mDiffuseReflectance;
    Vec3f mPhongReflectance;
    float mPhongExponent;
};
