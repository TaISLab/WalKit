/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef LASERPROCESSOR_HH
#define LASERPROCESSOR_HH

#include <unistd.h>
#include <math.h>
#include <list>
#include <set>
#include <vector>
#include <map>
#include <utility>
#include <algorithm>
#include <bullet/LinearMath/btVector3.h>

// OpenCV related Headers
#include <opencv2/core/core.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/ml.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/core/core_c.h> 

#include <tf2_ros/buffer.h>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "std_msgs/msg/header.hpp"

// Custom Messages related Headers
#include "walker_msgs/msg/step_stamped.hpp"


/** @brief A namespace containing the laser processor helper classes */
namespace laser_processor
{
    /**
    * @brief A struct representing a single sample (i.e., scan point) from the laser.
    */
    class Sample
    {
        public:
            int   index;
            float range;
            float intensity;
            float x;
            float y;

            /**
            * @brief Return pointer to sample of index <ind>
            */
            static Sample* Extract(int ind, const sensor_msgs::msg::LaserScan& scan);
    };

    /**
    * @brief The comparator structure allowing the creation of an ordered set of Samples
    */
    struct CompareSample
    {
        /**
        * @brief The comparator allowing the creation of an ordered set of Samples
        */  
        inline bool operator() (const Sample* a, const Sample* b) const
        {
            return (a->index <  b->index);
        }
    };

    /**
    * @brief An ordered set of Samples
    *
    * Ordered based on sample index
    */
    class SampleSet : public std::set<Sample*, CompareSample>
    {
        public:
            /**
            * @brief Destructor
            */
            ~SampleSet() { clear(); }

            /**
            * @brief Delete all pointers to samples in the set
            */
            void clear();

            /**
            * @brief Get the centroid of the sample points
            * @return Centriod in (x,y,0) (z-element assumed 0)
            */
            geometry_msgs::msg::Point getPosition() const;
           
    };

    /**
    * @brief A scan processor to split the scan into clusters
    */
    class ScanProcessor
    {   
        std::list<SampleSet*> clusters_;
        sensor_msgs::msg::LaserScan scan_;
        
        // feture related attributes
        int feat_count_;
        cv::Ptr<cv::ml::RTrees> forest;
        CvMat* tmp_mat;


        public:
            /**
            * @brief Get all the clusters in the scan
            * @return List of clusters
            */
            std::list<SampleSet*>& getClusters() { return clusters_; }

            /**
            * @brief Constructor
            * @param scan Scan to be processed
            */
            ScanProcessor();

            void setScan(const sensor_msgs::msg::LaserScan& scan);
            
            void setForestFile(std::string forest_file);
            
            /**
            * @brief Destructor
            */
            ~ScanProcessor();

            /**
            * @brief Remove and delete all references to scan clusters less than a minimum size
            * @param num Minimum number of points in cluster
            */
            void removeLessThan(uint32_t num);

            /**
            * @brief Split scan into clusters
            * @param thresh Euclidian distance threshold for clustering
            */
            void splitConnected(float thresh);

            /**
            * @brief Get number of clusters
            */
            int size(){ return clusters_.size(); };

            void removeFar(float max_dist);
            std::list<walker_msgs::msg::StepStamped> getCentroids(std_msgs::msg::Header scan_header);
    };
}

#endif
