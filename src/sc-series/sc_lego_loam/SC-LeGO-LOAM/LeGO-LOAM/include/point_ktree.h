#pragma once

#include <glog/logging.h>

#include <memory>
#include <vector>

#include "nanoflann/nanoflann.hpp"

// =======
// typedef std::vector<std::vector<double> > my_vector_of_vectors_t;
// typedef std::vector<Eigen::VectorXd> my_vector_of_vectors_t;
// =====================================================================================

/** A simple vector-of-vectors adaptor for nanoflann, without duplicating the
 * storage. The i'th vector represents a point in the state space.
 *
 *  \tparam DIM If set to >0, it specifies a compile-time fixed dimensionality
 * for the points in the data set, allowing more compiler optimizations. \tparam
 * num_t The type of the point coordinates (typically, double or float). \tparam
 * Distance The distance metric to use: nanoflann::metric_L1,
 * nanoflann::metric_L2, nanoflann::metric_L2_Simple, etc. \tparam IndexType The
 * type for indices in the KD-tree index (typically, size_t of int)
 */
template <class VectorOfVectorsType, typename num_t = double, int DIM = -1,
          class Distance = nanoflann::metric_L2, typename IndexType = size_t>
class PointVectorIndexData {
  typedef PointVectorIndexData<VectorOfVectorsType, num_t, DIM, Distance>
      AdaptorType;
  typedef typename Distance::template traits<num_t, AdaptorType>::distance_t
      MetricType;
  typedef nanoflann::KDTreeSingleIndexAdaptor<MetricType, AdaptorType, DIM,
                                              IndexType>
      AdaptorIndexType;

 public:
  PointVectorIndexData(const size_t /* dimensionality */,
                       const VectorOfVectorsType &mat,
                       const int leaf_max_size = 10)
      : data_(mat) {
    CHECK(mat.size() != 0 && mat[0].size() != 0);
    const size_t dims = mat[0].size();
    if (DIM > 0 && static_cast<int>(dims) != DIM) {
      throw std::runtime_error(
          "Data set dimensionality does not match the 'DIM' template argument");
    }

    index_ = std::make_shared<AdaptorIndexType>(
        static_cast<int>(dims), *this /* adaptor */,
        nanoflann::KDTreeSingleIndexAdaptorParams(leaf_max_size));
    index_->buildIndex();
  }

  std::shared_ptr<AdaptorIndexType> Index() { return index_; }
  const AdaptorType &derived() const { return *this; }
  AdaptorType &derived() { return *this; }

  inline void Query(const num_t *query_point, const size_t num_closest,
                    IndexType *out_indices, num_t *out_distances_sq,
                    const int nChecks_IGNORED = 10) const {
    nanoflann::KNNResultSet<num_t, IndexType> resultSet(num_closest);
    resultSet.init(out_indices, out_distances_sq);
    index_->findNeighbors(resultSet, query_point, nanoflann::SearchParams());
  }

  inline size_t kdtree_get_point_count() const { return data_.size(); }

  inline num_t kdtree_get_pt(const size_t idx, const size_t dim) const {
    return data_[idx][dim];
  }

  template <class BBOX>
  bool kdtree_get_bbox(BBOX & /*bb*/) const {
    return false;
  }

 private:
  std::shared_ptr<AdaptorIndexType> index_;
  const VectorOfVectorsType &data_;
};
