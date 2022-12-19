#pragma once

#include <vector>
#include <cmath>
#include <omp.h>
#include <cassert>
#include "renderer.hpp"
#include "rng.hpp"

class PathTracer : public AbstractRenderer
{
public:

    PathTracer(
        const Scene& aScene,
        int aSeed = 1234
    ) :
        AbstractRenderer(aScene), mRandomGenerator(aSeed)
    {}

    virtual void RunIteration(int iteration)
    {
        const int resolutionX = int(mScene.mCamera.mResolution.x);
        const int resolutionY = int(mScene.mCamera.mResolution.y);

        for (int pixelID = 0; pixelID < resolutionX * resolutionY; pixelID++)
        {
            // Current pixel coordinates (as integers):
            const int x = pixelID % resolutionX;
            const int y = pixelID / resolutionX;

            // Current pixel coordinates (as floating point numbers, randomly positioned inside a pixel square):
            // E.g., for x = 5, y = 12, we can have sample coordinates from x = 5.00 to 5.99.., and y = 12.00 to 12.99..
            const Vec2f sample = Vec2f(float(x), float(y)) + mRandomGenerator.GetVec2f();

            // Generating a ray with an origin in the camera with a direction corresponding to the pixel coordinates:
            Ray ray = mScene.mCamera.GenerateRay(sample);

            auto intersection = mScene.FindClosestIntersection(ray);
            if (intersection)
            {
				const Vec3f surfacePoint = ray.origin + ray.direction * intersection->distance;
				CoordinateFrame frame;
				frame.SetFromZ(intersection->normal);
				const Vec3f incomingDirection = frame.ToLocal(-ray.direction);

                Vec3f LoDirect = Vec3f(0);
                Vec3f LoEmitted = Vec3f(0);

				const Material& mat = mScene.GetMaterial(intersection->materialID);
                // Direct intersection with the light source
                const int lightId = intersection->lightID;
                if (lightId >= 0) {
                    LoEmitted += mScene.GetLightPtr(lightId)->Evaluate(incomingDirection);
                    mFramebuffer.AddColor(sample, LoEmitted + LoDirect);
                    
                    // This turns out to be important to cull these ray samples. 
                    // Figured this out when testing for BRDF-importance sampling??
                    continue;
                }

                // ------------------------------------------------------------------------
                // Otherwise intesected something other than a direct light source
                // ------------------------------------------------------------------------

                // Sample reflected direction, single bounce, BRDF-importance sampling
                {
                    // Memo: Both directions are pointed outwards and in local coordinates
                    auto [outgoingDirectionLocal, reflectedIntensity, _] = mat.SampleReflectedDirection(incomingDirection, mRandomGenerator);
                    auto pdf = mat.PDF(incomingDirection, outgoingDirectionLocal);
                    Vec3f outgoingDirection = Normalize(frame.ToWorld(outgoingDirectionLocal));
                    float cosTheta = Dot(frame.mZ, outgoingDirection);

                    // Cast a ray in the sampled direction
                    Vec3f intensity = Vec3f(0.f);
                    float weightFactorMIS = 0.f;
                    auto pdfOther = 0.f;
                    Ray raySample(surfacePoint, outgoingDirection, EPSILON_RAY);
                    if (!mScene.FindAnyIntersection(raySample))
                    {
                        // No ray-object intersection occurs, try evaluating the background light
                        if (mScene.GetBackground()) {
                            weightFactorMIS = -1;
                            intensity = mScene.GetBackground()->Evaluate(outgoingDirection);
                            pdfOther = mScene.GetBackground()->PDF(surfacePoint, Vec3f(0.f));
                        }
                    } else {
                        int intersectedLightId = mScene.FindClosestIntersection(raySample)->lightID;
                        if (intersectedLightId < 0) {
                            // Intersected something else, ignore for now?
                        } else {
                            // Ray bounce intersected the light, we add the contribution of Lo_ from that light source
                            weightFactorMIS = -1;
                            intensity = mScene.GetLightPtr(intersectedLightId)->Evaluate(outgoingDirection);
                            Vec3f intersectionPoint = mScene.FindClosestIntersection(raySample)->distance * outgoingDirection + surfacePoint;
                            pdfOther = mScene.GetLightPtr(intersectedLightId)->PDF(surfacePoint, intersectionPoint);
                        }
                    }

                    // Calculate the weight factor for MIS, if prompted for re-evaluation
                    if (weightFactorMIS == -1)
                    {
                        weightFactorMIS = pdf / (pdf + pdfOther);
                    }
                    
                    LoDirect += weightFactorMIS * intensity * mat.EvaluateBRDF(Normalize(outgoingDirectionLocal), incomingDirection) * cosTheta / pdf;
                }
                
                // Sampling the light sources directly
                // Connect from the current surface point to every light source in the scene:
                for (int i = 0; i < mScene.GetLightCount(); i++)
                {
                    const AbstractLight* light = mScene.GetLightPtr(i);
                    assert(light != 0);

                    float weightFactorMIS = 0.f;
                    auto pdfOther = 0.f;

                    auto [lightPoint, intensity, pdf] = light->SamplePointOnLight(surfacePoint, mRandomGenerator);
                    Vec3f outgoingDirection = Normalize(lightPoint - surfacePoint);
                    float lightDistance = sqrt((lightPoint - surfacePoint).LenSqr());
                    float cosTheta = Dot(frame.mZ, outgoingDirection);

                    if (cosTheta > 0 && intensity.Max() > 0)
                    {
                        Ray rayToLight(surfacePoint, outgoingDirection, EPSILON_RAY); // Note! To prevent intersecting the same object we are already on, we need to offset the ray by EPSILON_RAY
                        if (!mScene.FindAnyIntersection(rayToLight, lightDistance)) { // Testing if the direction towards the light source is not occluded
                            pdfOther = mat.PDF(incomingDirection, frame.ToLocal(outgoingDirection));
                            weightFactorMIS = pdf / (pdf + pdfOther);

                            // Exception case, point light pdf 100%
                            if (pdf == 1.f) {
                                weightFactorMIS = 1.f;
                            }

                            LoDirect += weightFactorMIS * intensity * mat.EvaluateBRDF(frame.ToLocal(outgoingDirection), incomingDirection) * cosTheta / pdf;
                        }
                    }
                }

                //printf("%d %d %d", LoDirect.x, LoDirect.y, LoDirect.z);

				mFramebuffer.AddColor(sample, LoEmitted + LoDirect);
            }
        }

        mIterations++;
    }

    Rng mRandomGenerator;
};
