#ifndef IG_GRAPH_SLAM_KDTREETYPE_HPP
#define IG_GRAPH_SLAM_KDTREETYPE_HPP

#include <nanoflann.hpp>

// This is a dataset class to hold init transform points
template <typename T> struct InitPose
{
    std::vector<Eigen::Affine3d> poses;

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const
    {
        return poses.size();
    }

    // Returns the distance between the vector "p1[0:size-1]" and the data point
    // with index "idx_p2" stored in the class:
    inline T kdtree_distance(const T *p1,
                             const size_t idx_p2,
                             size_t /*size*/) const
    {
        const T d0 = p1[0] - poses[idx_p2](0, 3);
        const T d1 = p1[1] - poses[idx_p2](1, 3);
        const T d2 = p1[2] - poses[idx_p2](2, 3);
        return d0 * d0 + d1 * d1 + d2 * d2;
    }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate
    // value, the
    //  "if/else's" are actually solved at compile time.
    inline T kdtree_get_pt(const size_t idx, int dim) const
    {
        if (dim == 0)
            return poses[idx](0, 3);
        else if (dim == 1)
            return poses[idx](1, 3);
        else
            return poses[idx](2, 3);
    }

    // Optional bounding-box computation: return false to default to a standard
    // bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned
    //   in
    //   "bb" so it can be avoided to redo it again.
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3
    //   for point clouds)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX & /* bb */) const
    {
        return false;
    }
};

using kd_tree_t = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<double, InitPose<double>>,
        InitPose<double>,
        3>;

#endif //IG_GRAPH_SLAM_KDTREETYPE_HPP
