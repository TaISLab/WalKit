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

#include <walker_step_detector/laser_processor.h>
#include "walker_step_detector/cluster_features.h"  // mfc: dont move me to header ...

namespace laser_processor 
{
    Sample* Sample::Extract(int ind, const sensor_msgs::msg::LaserScan& scan){

        Sample* s = new Sample();

        s->index = ind;
        s->range = scan.ranges[ind];
        s->x = cos( scan.angle_min + ind*scan.angle_increment ) * s->range;
        s->y = sin( scan.angle_min + ind*scan.angle_increment ) * s->range;
        if (s->range > scan.range_min && s->range < scan.range_max)
        {
            return s;
        }
        else
        {
            delete s;
            return NULL;
        }
    }

    
    void SampleSet::clear(){

        for (SampleSet::iterator i = begin(); i != end(); ++i)
            delete (*i);

        std::set<Sample*, CompareSample>::clear();

    }

    geometry_msgs::msg::Point SampleSet::getPosition() const{

        geometry_msgs::msg::Point point;
        float x_mean = 0.0;
        float y_mean = 0.0;
        for (iterator i = begin(); i != end(); ++i)
        {
            x_mean += ((*i)->x)/size();
            y_mean += ((*i)->y)/size();
        }
        point.x = x_mean; 
        point.y = y_mean;
        point.z = 0.0;
        return point;
    }


    ScanProcessor::ScanProcessor(){

    }
    
    void ScanProcessor::setScan(const sensor_msgs::msg::LaserScan& scan){
        scan_ = scan;

        // remove any previous data
        std::list<SampleSet*>::iterator c_iter = clusters_.begin();
        while (c_iter != clusters_.end())
        {
                delete (*c_iter);
                clusters_.erase(c_iter++);
        }

        // add new ones
        SampleSet* cluster = new SampleSet;
        for (unsigned long int i = 0; i < scan.ranges.size(); i++)
        {
            Sample* s = Sample::Extract(i, scan);

            if (s != NULL)
            cluster->insert(s);
        }

        clusters_.push_back(cluster);
    }
            
    
    
    void ScanProcessor::setForestFile(std::string forest_file){

        //forest = cv::ml::RTrees::create();       
        
        //Load Random forest
        forest = cv::ml::StatModel::load<cv::ml::RTrees>(forest_file);
        feat_count_ = forest->getVarCount();

        // OpenCV matrix needed to use the OpenCV random forest classifier
        tmp_mat = cvCreateMat(1, feat_count_, CV_32FC1);

    }

    ScanProcessor::~ScanProcessor(){
        cvReleaseMat(&tmp_mat);
        for ( std::list<SampleSet*>::iterator c = clusters_.begin(); c != clusters_.end(); ++c)
            delete (*c);

    }

    void ScanProcessor::removeLessThan(uint32_t num){

        std::list<SampleSet*>::iterator c_iter = clusters_.begin();
        while (c_iter != clusters_.end())
        {
            if ( (*c_iter)->size() < num )
            {
                delete (*c_iter);
                clusters_.erase(c_iter++);
            } 
            else 
            {
                ++c_iter;
            }
        }
    }

    /*
    TODO this does not work
    I should get rid of all those pointers I can't control and redo all this ...
    void ScanProcessor::removeLines(double threshold, rclcpp::Node *boss){

        std::list<SampleSet*>::iterator c_iter = clusters_.begin();
        RCLCPP_ERROR(boss->get_logger(), "Removing lines"); 
        // Iterate over all clusters
        while (c_iter != clusters_.end())
        {
            RCLCPP_ERROR(boss->get_logger(), "Reading cluster"); 
            // Iterate over laser scan samples in each cluster
            SampleSet* cluster_i = *c_iter;            
            RCLCPP_ERROR(boss->get_logger(), "Cluster has %ld samples",cluster_i->size()); 
            if (cluster_i->size() > 0)
            {                
                double m_mean = 0;
                double m_var = 0;
                double i = 1;
                SampleSet::iterator s_iter = cluster_i->begin();
                Sample *s1 = *s_iter;
                s_iter++;
                RCLCPP_ERROR(boss->get_logger(), "Procesing samples"); 
                while (s_iter != cluster_i->end()){
                    Sample *s2 = *s_iter;
                    double m = (s2->y - s1->y)/(s2->x - s1->x);

                    m_mean = m/i + (i-1)*m_mean/i;
                    m_var  = pow(m-m_mean,2)/i + (i-1)*m_var/i;

                    s1 = s2;
                    i = i + 1.0;
                    s_iter++;
                }

                if (m_var<threshold){
                    RCLCPP_ERROR(boss->get_logger(), "Deleting line %3.3f : %3.3f ---------------------------------------------", m_mean, m_var);
                    //delete (*c_iter);
                    clusters_.erase(c_iter);
                    RCLCPP_ERROR(boss->get_logger(), "\n\n\n\n\n");                
                }
            }

            c_iter++;
        }


        RCLCPP_ERROR(boss->get_logger(), "Done"); 


    }
    */
    void ScanProcessor::splitConnected(float thresh){

        // Holds our temporary list of split clusters 
        // because we will be modifying our existing list in the mean time
        std::list<SampleSet*> tmp_clusters;

        std::list<SampleSet*>::iterator c_iter = clusters_.begin();

        while (c_iter != clusters_.end())
        {

            while ((*c_iter)->size() > 0)
            {

                // Iterate over laser scan samples in clusters_
                // and collect those which are within a euclidian distance of <thresh>
                // and store new clusters in tmp_clusters
                SampleSet::iterator s_first = (*c_iter)->begin();
                std::list<Sample*> sample_queue;
                sample_queue.push_back(*s_first);
                (*c_iter)->erase(s_first);
                std::list<Sample*>::iterator s_q = sample_queue.begin();

                while (s_q != sample_queue.end()){

                    int expand = (int)(asin( thresh / (*s_q)->range ) / scan_.angle_increment);

                    SampleSet::iterator s_rest = (*c_iter)->begin();

                    while ( (s_rest != (*c_iter)->end() and (*s_rest)->index < (*s_q)->index + expand ) ){

                        if (sqrt( pow( (*s_q)->x - (*s_rest)->x, 2.0f) + pow( (*s_q)->y - (*s_rest)->y, 2.0f)) < thresh){

                            sample_queue.push_back(*s_rest);
                            (*c_iter)->erase(s_rest++);
                        }
                        else {
                            ++s_rest;
                        }    
                    }
                    s_q++;
                }
                // Move all the samples into the new cluster
                SampleSet* c = new SampleSet;
                for (s_q = sample_queue.begin(); s_q != sample_queue.end(); s_q++)
                    c->insert(*s_q);

                // Store the temporary clusters
                tmp_clusters.push_back(c);
            }

            //Now that c_iter is empty, we can delete
            delete (*c_iter);

            //And remove from the map
            clusters_.erase(c_iter++);
        }

        // Insert our temporary clusters list back into the de facto list
        clusters_.insert(clusters_.begin(), tmp_clusters.begin(), tmp_clusters.end());
    }

    void ScanProcessor::removeFar(float max_dist){
        geometry_msgs::msg::Point point;
        float rel_dist = 4092.0;
        std::list<SampleSet*>::iterator c_iter = clusters_.begin();
        while (c_iter != clusters_.end()){
            point = (*c_iter)->getPosition();

            rel_dist = 4092.0;
            try {
                rel_dist = pow(point.x*point.x + point.y*point.y, 1./2.);
            } catch (tf2::TransformException &e){
                //RCLCPP_ERROR (this->get_logger(), "%s", e.what());
            }

            if ( rel_dist > max_dist ) {
                delete (*c_iter);
                clusters_.erase(c_iter++);
            } else {
                ++c_iter;
            }
        }
    }

    std::list<walker_msgs::msg::StepStamped> ScanProcessor::getCentroids(std_msgs::msg::Header scan_header){
        std::list<walker_msgs::msg::StepStamped> centroids;

        ClusterFeatures cf_;
        
        std::list<SampleSet*>::iterator c_iter = clusters_.begin();
        while (c_iter != clusters_.end()){
            walker_msgs::msg::StepStamped new_step;
            new_step.position.header = scan_header;
            new_step.position.point = (*c_iter)->getPosition();

            // Add features: TODO REVISIT THESE FILEs....
            std::vector<float> f = cf_.calcClusterFeatures(*c_iter, scan_);
            for (int k = 0; k < feat_count_; k++)
                tmp_mat->data.fl[k] = (float)(f[k]);
            
            // Output of forest->predict is [-1.0, 1.0] so we scale to reach [0.0, 1.0]
            new_step.confidence = 0.5 * (1.0 + forest->predict(cv::cvarrToMat(tmp_mat)));

            centroids.push_back(new_step);
            
            ++c_iter;
        }
        return centroids;
    }

} // namespace laser_processor 

