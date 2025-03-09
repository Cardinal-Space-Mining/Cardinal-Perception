#include "perception.hpp"


template class csm::perception::KFCMap<
                    csm::perception::MappingPointType,
                    csm::perception::MapOctree<
                        csm::perception::MappingPointType,
                        void >,
                    csm::perception::CollisionPointType >;

template class csm::perception::LidarFiducialDetector<
                    csm::perception::FiducialPointType >;

template class csm::perception::TransformSynchronizer<
                    util::geom::Pose3d >;
