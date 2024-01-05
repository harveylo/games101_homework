#include <algorithm>
#include <cassert>
#include <cstring>
#include <iostream>
#include "BVH.hpp"
#include "Bounds3.hpp"
#include "Vector.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

static void sort_objects_by_dim(std::vector<Object*>& objects, int dim)
{
    switch (dim) {
    case 0:
        std::sort(objects.begin(), objects.end(), [](auto& f1, auto& f2) {
            return f1->getBounds().Centroid().x <
                   f2->getBounds().Centroid().x;
        });
        break;
    case 1:
        std::sort(objects.begin(), objects.end(), [](auto& f1, auto& f2) {
            return f1->getBounds().Centroid().y <
                   f2->getBounds().Centroid().y;
        });
        break;
    case 2:
        std::sort(objects.begin(), objects.end(), [](auto& f1, auto& f2) {
            return f1->getBounds().Centroid().z <
                   f2->getBounds().Centroid().z;
        });
        break;
    }
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        // Union function take two bounds, get a bound that can contain those two bounds completely
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        // if only two objects remains, then the bounds of the node is the union of the two objects
        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                // Centroid function returns the center point(Vector3f) of a bound
                // Bounds can be constructed from a Vector3f, in such case, the bound's min and max are both the Vector3f
                // which means the bound only encloses a point
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        // the final centroidBounds is the bound that encloses all the centroids of the objects.

        // ! SAH

        // Get the surface area of the whole bounding box
        auto SC = centroidBounds.SurfaceArea();

        // each spliting, n buckets are created
        int n = 12;
        // min cost and the corresponding spliting dimension
        auto minCost = std::numeric_limits<float>::max();
        auto minDim = 0;
        auto minIndex = 0;

        // split on each dimension to get the optimal spliting plan
        for(int dim = 0;dim<3;dim++){
            sort_objects_by_dim(objects, dim);
            Bounds3 cost_of_buckets[n];
            int count_of_buckets[n];
            for(auto& i : count_of_buckets) i = 0;
            auto min_t = centroidBounds.pMin[dim];
            auto max_t = centroidBounds.pMax[dim];

            // ! CAUTION: can not simply split the scene on objects
            // ! should assign each object to a bucket, then calculate the cost of each bucket
            for(auto object : objects){
                // calculate the index of the bucket that the object belongs to
                auto t = object->getBounds().Centroid()[dim];
                int index = (t - min_t) / (max_t - min_t) * n;
                if(index == n) index--;
                if(index < 0) index = 0;
            
                // update the bounds of the bucket
                cost_of_buckets[index] = Union(cost_of_buckets[index],object->getBounds());
                count_of_buckets[index]++;
            }

            auto min_cost_now = std::numeric_limits<float>::max();
            int min_index_now = 0;

            // ! there is n-1 spliting plans
            for (int i = 0; i < n-1; ++i){
                Bounds3 left, right;
                int count_left = 0, count_right = 0;
                // ! i is not included in the left part
                for(int j = 0;j<i;j++){
                    left = Union(left, cost_of_buckets[j]);
                    count_left += count_of_buckets[j];
                }
                for(int j = i;j<n;j++){
                    right = Union(right, cost_of_buckets[j]);
                    count_right += count_of_buckets[j];
                }
                // calculate the cost of the spliting plan
                float cost_left = left.SurfaceArea() / SC * count_left;
                float cost_right = right.SurfaceArea() / SC * count_right;
                float cost_now = cost_left + cost_right;

                if(cost_now < min_cost_now){
                    min_cost_now = cost_now;
                    min_index_now = i;
                }
            }

            if (min_cost_now < minCost){
                // the min_index_now is the index of the bucket that the spliting plan is based on
                // ! but what we need here is the index of the first object in the right part
                // ! thus we need to calculate the object id from the min_index_now
                int objects_id = 0;
                // ! recall that the min_index_now is not included in the left part
                for(int i = 0;i<min_index_now;i++) objects_id += count_of_buckets[i];
                minCost = min_cost_now;
                minDim = dim;
                minIndex = objects_id;
            }
        }

        // get the final spliting plan
        sort_objects_by_dim(objects, minDim);

        // split the objects into two vectors, each contains half of the objects
        auto leftshapes = std::vector<Object*>(objects.begin(), objects.begin() + minIndex);
        auto rightshapes = std::vector<Object*>(objects.begin() + minIndex, objects.end());

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        // recursively build the left and right nodes of each half
        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node -> bounds = Union(node->left->bounds, node->right->bounds);


        // ! Following codes are used to split objects in a normal manner
        // // the maxExtent function returns the dimension(x, y, z) of the bound that has the largest extent
        // // return 0 for x, 1 for y, 2 for z
        // int dim = centroidBounds.maxExtent();

        // // split the objects into two groups according to the dimension that has the largest extent
        // switch (dim) {
        // case 0:
        //     // sort objects on that dimension
        //     std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
        //         return f1->getBounds().Centroid().x <
        //                f2->getBounds().Centroid().x;
        //     });
        //     break;
        // case 1:
        //     std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
        //         return f1->getBounds().Centroid().y <
        //                f2->getBounds().Centroid().y;
        //     });
        //     break;
        // case 2:
        //     std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
        //         return f1->getBounds().Centroid().z <
        //                f2->getBounds().Centroid().z;
        //     });
        //     break;
        // }

        // auto beginning = objects.begin();
        // auto middling = objects.begin() + (objects.size() / 2);
        // auto ending = objects.end();

        // // split the objects into two vectors, each contains half of the objects
        // auto leftshapes = std::vector<Object*>(beginning, middling);
        // auto rightshapes = std::vector<Object*>(middling, ending);

        // assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        // // recursively build the left and right nodes of each half
        // node->left = recursiveBuild(leftshapes);
        // node->right = recursiveBuild(rightshapes);

        // // the bound of the node is the union of the bounds of the left and right nodes
        // node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    Intersection isect;
    if (!node->bounds.IntersectP(ray, ray.direction_inv))
        return isect;

    if (node->left == nullptr && node->right == nullptr) {
        isect = node->object->getIntersection(ray);
        return isect;
    }

    Intersection left = getIntersection(node->left, ray);
    Intersection right = getIntersection(node->right, ray);

    return left.distance < right.distance ? left : right;
}