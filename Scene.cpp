//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"
#include "Intersection.hpp"
#include "Vector.hpp"
#include "global.hpp"



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

    // ? Get the intersection information of the ray
    Intersection hit_intersec = intersect(ray);
    // if hit nothing, return the background color
    // if(!hit_intersec.happened) return this->backgroundColor;
    if(!hit_intersec.happened) return {0,0,0};

    auto shade_point = hit_intersec.coords;
    auto material = *(hit_intersec.m);
    auto normal = hit_intersec.normal;
    auto wo = -ray.direction.normalize();

    // the point that will be used as the origin of child ray
    // ! must be slightly away from the hit point in case the child ray hit the same object again
    // the sign of the offset is determined by the direction of the ray and the normal of the hit point
    // which means, whether the ray hits the front face or the back face of the object
    auto shade_point_calibrated = dotProduct(wo, normal)>0? shade_point + normal * EPSILON : shade_point - normal * EPSILON;

    Vector3f direct_light {0,0,0};
    // ? sample the direct light
    Intersection light_intersec;

    float light_pdf;
    sampleLight(light_intersec, light_pdf);
    auto light_pos = light_intersec.coords;
    auto light_normal = light_intersec.normal;
    auto light_wi = (light_pos - shade_point).normalize();
    auto cos_light_theta = dotProduct(normal, light_wi);
    auto cos_light_theta_prime = dotProduct(light_normal, -light_wi);  
    auto light_point_distance = (light_pos - shade_point).length();

    // ? test if the light is blocked
    Ray shadow_ray(shade_point_calibrated, light_wi);
    Intersection shadow_intersec = intersect(shadow_ray);

    // if the light is not blocked, add the direct light
    if(std::abs(shadow_intersec.distance-light_point_distance) < 0.001){
        direct_light = light_intersec.emit*
                        material.eval(wo, light_wi, normal) * 
                        cos_light_theta * cos_light_theta_prime / (light_point_distance * light_point_distance) / light_pdf;
    }

    // ? sample the indirect light
    Vector3f indirect_light {0,0,0};

    auto rr = get_random_float();
    if(rr <= RussianRoulette){
        // get a random direction by the material's sample function
        auto wi = material.sample(wo, normal);

        // generate the child ray
        Ray indirect_ray(shade_point_calibrated, wi);

        auto cos_theta = std::max(0.0f, dotProduct(normal, wi));
        auto pdf_value = material.pdf(wo, wi, normal);
        auto f_r = material.eval(wo, wi, normal);

        auto indir_intersec = intersect(indirect_ray);
        // ! if the child ray hit a light source all nothing, no need to calculate
        // ! because we already have the direct light sampled
        // ! also, if the pdf value is too small, we consider such ray as a noise
        if(indir_intersec.happened && !indir_intersec.m->hasEmission() && pdf_value > EPSILON){
            // get the indirect light
            indirect_light = castRay(indirect_ray, depth + 1) * f_r * cos_theta / pdf_value / RussianRoulette;
        }

    }

    // ! remember to add the emission of the hit-point's own material
    return material.getEmission()+direct_light + indirect_light;
}