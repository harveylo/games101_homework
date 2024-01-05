The SAH is implemented based on three steps:

- split the scene into $n$ buckets on each axis
- calculate each bucket's bound
- find out the best split plan on which axis and bucket

The key part is how to calculate the cost based on surface area.
Please refer to `BVH.cpp:91` for more information.
