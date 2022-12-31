#pragma once

// TODO: list
/*
* 1. restructure how the ray is being generated
* 2. Form it into a method based on that ray
* 3. Form it into a primitive path tracer first, to test out how it works
*/


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

    virtual Vec3f EstimateLin(Ray ray) 
    {
        Vec3f accumulation = Vec3f(0);
        Vec3f throughput = Vec3f(1);

		int bounces = 0;
		while(1)
		{
            Vec3f origin = ray.origin;
            Vec3f direction = ray.direction;

            auto intersection = mScene.FindClosestIntersection(ray);

            if (!intersection)
            {
				// Sample the environment map if no scene intersection (this handles scenes with envMaps)
				Vec3f backgroundLe = Vec3f(0);
				if (mScene.GetBackground()) {
					backgroundLe = mScene.GetBackground()->Evaluate(-ray.direction);
				}
                return accumulation;// The factor is accounted from MIS? +throughput * backgroundLe;
            }

			// Initial ray intersects the scene
			const Vec3f surfacePoint = ray.origin + ray.direction * intersection->distance;
			CoordinateFrame frame;
			frame.SetFromZ(intersection->normal);
			const Vec3f incomingDirection = frame.ToLocal(-ray.direction);

			const Material& mat = mScene.GetMaterial(intersection->materialID);
			// Path intersection with the light source
			const int lightId = intersection->lightID;
			if (lightId >= 0)
			{
				Vec3f LoEmitted = mScene.GetLightPtr(lightId)->Evaluate(incomingDirection);
				// Condition ? Only first bounce counts towards the contribution (Meaning direct light source intersection ONLY)
				if(bounces == 0)
				{
					accumulation += throughput * LoEmitted;
				}
				return accumulation;
				// TODO: Difference is that we don't stop the method, unless the survival rate dies off.
			}

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
                }
                else {
                    int intersectedLightId = mScene.FindClosestIntersection(raySample)->lightID;
                    if (intersectedLightId < 0) {
                        // Intersected something else, ignore for now?
                    }
                    else {
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

                accumulation += throughput * weightFactorMIS * intensity * mat.EvaluateBRDF(Normalize(outgoingDirectionLocal), incomingDirection) * cosTheta / pdf;
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

                        accumulation += throughput * weightFactorMIS * intensity * mat.EvaluateBRDF(frame.ToLocal(outgoingDirection), incomingDirection) * cosTheta / pdf;
                    }
                }
            }

			// Random bounce in the scene (propagating the ray)
			{
				// Both directions are pointed outwards and in local coordinates
				auto [outgoingDirectionLocal, reflectedIntensity, _] = mat.SampleReflectedDirection(incomingDirection, mRandomGenerator);
				auto pdf = mat.PDF(incomingDirection, outgoingDirectionLocal);
				Vec3f outgoingDirection = Normalize(frame.ToWorld(outgoingDirectionLocal));
				float cosTheta = Dot(frame.mZ, outgoingDirection);

				throughput *= mat.EvaluateBRDF(Normalize(outgoingDirectionLocal), incomingDirection) * cosTheta / pdf;

				// The survival probability roll
				float rNum = mRandomGenerator.GetFloat();
				float survivalProb = std::min(1.f, maxComponent(throughput));
				if (rNum < survivalProb) 
				{
					throughput /= survivalProb;
					Ray raySample(surfacePoint, outgoingDirection, EPSILON_RAY);
					ray = raySample;
				}
				else 
				{
					break;
				}

				//// Cast a ray in the sampled direction
				//Vec3f intensity = Vec3f(0.f);
				//Ray raySample(surfacePoint, outgoingDirection, EPSILON_RAY);
				//if (!mScene.FindAnyIntersection(raySample)) {
				//	//TODO: ? // No ray-object intersection occurs, try evaluating the background light
				//	//if (mScene.GetBackground()) {
				//	//	intensity = mScene.GetBackground()->Evaluate(outgoingDirection);
				//	//}
				//}
				//else
				//{
				//	// Some scene intersection occurs
				//	auto intersectionInfo = mScene.FindClosestIntersection(raySample);
				//	int intersectedLightId = intersectionInfo->lightID;
				//	if (intersectedLightId < 0) {
				//		// Intersected something else, ignore for now?
				//	}
				//	else {
				//		// Ray bounce intersected the light, we add the contribution of Lo_ from that light source
				//		intensity = mScene.GetLightPtr(intersectedLightId)->Evaluate(outgoingDirection);
				//	}
				//}
				//accumulation += intensity * mat.EvaluateBRDF(Normalize(outgoingDirectionLocal), incomingDirection) * cosTheta / pdf;
			}

			/// ---------- Sampling the light sources directly
			// Connect from the current surface point to every light source in the scene:
			/*for (int i = 0; i < mScene.GetLightCount(); i++)
			{
				const AbstractLight* light = mScene.GetLightPtr(i);
				assert(light != 0);

				auto [lightPoint, intensity, pdf] = light->SamplePointOnLight(surfacePoint, mRandomGenerator);
				Vec3f outgoingDirection = Normalize(lightPoint - surfacePoint);
				float lightDistance = sqrt((lightPoint - surfacePoint).LenSqr());
				float cosTheta = Dot(frame.mZ, outgoingDirection);

				if (cosTheta > 0 && intensity.Max() > 0)
				{
					Ray rayToLight(surfacePoint, outgoingDirection, EPSILON_RAY); // Note! To prevent intersecting the same object we are already on, we need to offset the ray by EPSILON_RAY
					if (!mScene.FindAnyIntersection(rayToLight, lightDistance)) { // Testing if the direction towards the light source is not occluded
						LoDirect += intensity * mat.EvaluateBRDF(frame.ToLocal(outgoingDirection), incomingDirection) * cosTheta / pdf;
					}
				}
			}*/

			//printf("%d %d %d", LoDirect.x, LoDirect.y, LoDirect.z);

			bounces++;
        }

        return accumulation;
    }

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

            Vec3f radianceSample = EstimateLin(ray);

            mFramebuffer.AddColor(sample, radianceSample);

        }

        mIterations++;
    }

    Rng mRandomGenerator;
};
