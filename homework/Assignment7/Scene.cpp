//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Vector3f result = 0;

//    if (depth > maxDepth) {
//        return result;
//    }

    Intersection interScene = intersect(ray); // ray = one eyesight
    if (interScene.happened) { // can see sth in this (ray's) direction

        Intersection lightPos;
        float pdf = 0.0f;
        sampleLight(lightPos, pdf); // get direct light sample

        Vector3f p = interScene.coords; // the point is being lighted/observed
        Vector3f x = lightPos.coords; // the real origin of the sampled light
        Vector3f N = interScene.normal.normalized(); // normal at the observed point
        Vector3f NN = lightPos.normal.normalized(); // normal at the lignt sample point
        Vector3f emit = lightPos.emit;  // the emission of the sampled light
        Vector3f ws = (x - p).normalized(); // direction from light sample to observed point (converse)
        Vector3f wo = ray.direction; // direction from observed point to observer's eye
        float dis = (x - p).norm();

        if (interScene.m->hasEmission()) {
            result += interScene.m->getEmission();
        } else {
            Ray pToX(p, ws);
            Intersection pToXInter = intersect(pToX);
            if (pToXInter.distance - (x - p).norm() > -0.01) {
                result += emit * interScene.m->eval(wo, ws, N)
                        * dotProduct(ws, N)
                        * dotProduct(-ws, NN)
                        / dis / dis / pdf;
            }

            if (get_random_float() <= RussianRoulette) {
                Vector3f wi = (interScene.m->sample(wo, N)).normalized();
                Ray pToWi(p, wi);
                Intersection pToWiInter = intersect(pToWi);
                if (pToWiInter.happened && (!pToWiInter.m->hasEmission())) {
                    result += castRay(pToWi, depth + 1)
                             * interScene.m->eval(wo, wi, N)
                             * dotProduct(wi, N)
                             / interScene.m->pdf(wo, wi, N)
                             / RussianRoulette;
                }
            }
        }
    }

    return result;
}