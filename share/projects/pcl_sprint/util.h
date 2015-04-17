template < typename PointT >
bool pcl_is_valid_point(const PointT & p, bool kinect = true) {
    /* if(pcl_isfinite(p.x)) return false; */
    /* if(pcl_isfinite(p.y)) return false; */
    /* if(pcl_isfinite(p.z)) return false; */
    if(kinect && p.z<0) return false;
    return true;
}
