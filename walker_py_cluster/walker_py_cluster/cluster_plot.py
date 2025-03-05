import rclpy
from rclpy.node import Node
import numpy as np
from visualization_msgs.msg import Marker
from squaternion import Quaternion
from math import pi
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan 
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2

from rclpy.qos import qos_profile_sensor_data
import time
import warnings

from sklearn import cluster
from sklearn.preprocessing import StandardScaler
from pykalman import AdditiveUnscentedKalmanFilter, UnscentedKalmanFilter
# pip3 install pykalman
# https://github.com/pykalman/pykalman/blob/master/examples/standard/plot_online.py


class ClusterPlot(Node):

    def __init__(self):
        super().__init__('cluster_plot')

        self.laser_topic_name= '/scan_feet'

        # ROS stuff
        self.laser_sub = self.create_subscription( LaserScan, self.laser_topic_name, self.listener_callback, qos_profile_sensor_data)      
        self.r_marker_pub = self.create_publisher(Path, "r_cluster", 10)
        self.l_marker_pub = self.create_publisher(Path, "l_cluster", 10)
        self.pcl_publisher_ = self.create_publisher(PointCloud2, 'test_cloud', 10)

        # create object
        spectral = cluster.SpectralClustering(
            n_clusters=2,
            eigen_solver='arpack', 
            n_components=None, 
            random_state=None, 
            n_init=10, 
            gamma=1.0, # Kernel coefficient for rbf, poly, sigmoid, laplacian and chi2 kernels. Ignored for affinity='nearest_neighbors'.
            affinity='nearest_neighbors',   # rbf, precomputed, precomputed_nearest_neighbors or any pairwise_kernel
            n_neighbors=10, 
            #eigen_tol='auto', 
            assign_labels='kmeans',  # kmeans discretize cluster_qr 
            degree=3,                # only for polynomial kernel
            coef0=1,                 # polynomial and sigmoid kernels. Ignored by others
            kernel_params=None,      # for callable kernels 
            n_jobs=2,                # parallel jobs if affinity='nearest_neighbors' or 'precomputed_nearest_neighbors'
            #verbose=False
        )

        # 3x slower than spectral
        km = cluster.KMeans(n_clusters=2, 
                            init='k-means++', 
                            #n_init='auto', 
                            max_iter=300, 
                            tol=0.0001, 
                            verbose=0, 
                            random_state=None, 
                            copy_x=True, 
                            algorithm='full' # 'auto', 'full' or 'elkan',
                            )
        
        # fixed number of clusters may not be best option
        mbkm = cluster.MiniBatchKMeans(n_clusters=3,
                                       init='k-means++', 
                                       max_iter=100, 
                                       batch_size=100, 
                                       verbose=0, 
                                       compute_labels=True, 
                                       random_state=None, 
                                       tol=0.0, 
                                       max_no_improvement=10, 
                                       init_size=None, 
                                       #n_init='auto', 
                                       reassignment_ratio=0.01
                                       )

        # got it. Now we need to deal with scenarios where clusters are not == 2 ... Kalman??
        agCl = cluster.AgglomerativeClustering(n_clusters=None,  # None if distance_threshold is not None
                                               #metric='euclidean', 
                                               memory=None, 
                                               connectivity=None, 
                                               compute_full_tree=True, 
                                               linkage='complete', 
                                               distance_threshold=0.15, 
                                               #compute_distances=False
                                               )    
        self.algorithm = agCl
        
        self.get_logger().info("Cluster plotter started")  

    def polar_to_cartesians(self,scan):
        theta = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges) )
        r = np.array(scan.ranges)
        x = np.cos(theta) * r
        y = np.sin(theta) * r

        # keep valid values
        valid_indexs = np.isfinite(scan.ranges)
        r[~valid_indexs] = scan.range_min - 0.1
        valid_indexs = np.logical_and(valid_indexs, r>=scan.range_min)
        valid_indexs = np.logical_and(valid_indexs, r<=scan.range_max)
        x = x[valid_indexs] 
        y = y[valid_indexs]

        # using index for segmentation ... not very helpful
        #z = np.where(valid_indexs)[0]
        #X = np.column_stack((x,y,z))
        X = np.column_stack((x,y))
        return X     

    def listener_callback(self, msg):
        t0 = time.time()
        # ....................................................
        
        Xi = self.polar_to_cartesians(msg)

        if len(Xi[:,0])>0:
            # fit data
            # catch warnings related to kneighbors_graph
            with warnings.catch_warnings():
                warnings.filterwarnings(
                    "ignore",
                    message="the number of connected components of the "
                    + "connectivity matrix is [0-9]{1,2}"
                    + " > 1. Completing it to avoid stopping the tree early.",
                    category=UserWarning,
                )
                warnings.filterwarnings(
                    "ignore",
                    message="Graph is not fully connected, spectral embedding"
                    + " may not work as expected.",
                    category=UserWarning,
                )
                self.algorithm.fit(Xi)

            # define labels
            if hasattr(self.algorithm, "labels_"):
                y_pred = self.algorithm.labels_.astype(int)
            else:
                y_pred = self.algorithm.predict(Xi)
            
            self.publish_paths(y_pred, Xi, msg.header)

        # ....................................................
        t1 = time.time()
        print( ("%3.3f ms" % (1000.0*(t1 - t0))).lstrip("0") )


    def publish_paths(self, labels, Xi, header):
        path_l = Path()
        path_l.header = header
        path_r = Path()
        path_r.header = header
        mean_y_r = 0
        mean_y_l = 0

        point_lists = dict()
        mean_y_lists = dict()

        # Tried to use index in clustering. not very helpful
        # str_label_0 = ""
        # str_label_1 = ""
        # str_t = ""
        # for i in range(0,len(labels)):
        #     if labels[i] == 0:
        #         str_label_0 = str_label_0 +  str(Xi[i,2]) +  "\t"
        #     else:
        #         str_label_1 = str_label_1 +  str(Xi[i,2]) +  "\t"
        # print("\n\nlabel 0")
        # print(str_label_0)
        # print("\n\nlabel 1")
        # print(str_label_1)

        for i in range(0,len(labels)):
            p = PoseStamped()
            p.header = header
            p.pose.orientation.w = 1.0
            p.pose.position.x = Xi[i,0]
            p.pose.position.y = Xi[i,1]
            label_i = labels[i]
            if label_i in point_lists.keys():
                point_lists[label_i].append(p)
            else:
                point_lists[label_i]= [p]

            if label_i in mean_y_lists.keys():
                mean_y_lists[label_i] = mean_y_lists[label_i] + Xi[i,1]
            else:
                mean_y_lists[label_i]= Xi[label_i,1]


        for label_i,mean_y_i in mean_y_lists.items():
            n_points = len(point_lists[label_i])
            if n_points>0:
                mean_y_lists[label_i] = mean_y_i/n_points
            else:
                mean_y_lists[label_i] = np.nan

        # in case we have more than 2 labels, we should, pick two 
        # sort dict by list length
        # point_lists = dict(sorted(point_lists.items(), key=lambda item: len(item[1])))
        path_l.poses = point_lists[0]
        path_r.poses = point_lists[1]
        mean_y_l = mean_y_lists[0]
        mean_y_r = mean_y_lists[1]

        if mean_y_l > mean_y_r:
            if len(path_r.poses)>0:
                self.r_marker_pub.publish(path_r)
            if len(path_l.poses)>0:
                self.l_marker_pub.publish(path_l)
        else:
            if len(path_l.poses)>0:
                self.r_marker_pub.publish(path_l)
            if len(path_r.poses)>0:
                self.l_marker_pub.publish(path_r)


        if hasattr(self.algorithm, "cluster_centers_"):
            C = self.algorithm.cluster_centers_
        else:
            C = self.get_centroids(labels, Xi)

        num_clusters = C.shape[0]
        x = []
        y = []
        labels = []

        for i in range(0,num_clusters):
            #print(C[i,:])
            x.append(C[i,0])
            y.append(C[i,1])
            labels.append(i)

        self.pcl_callback(header,x,y,labels)
    
    def get_centroids(self,labels, X):
        unique_labels = np.unique(labels)
        num_clusters = len(unique_labels)
        num_points = len(labels)

        centroids = np.zeros((num_clusters,2))

        for i in range(0,len(unique_labels)):
            label_i = unique_labels[i]
            mask_label = np.where(labels==label_i,1,0).reshape(1,num_points)
            num_label_i = mask_label.sum()
            c = mask_label.dot(X) /num_label_i
            centroids[label_i,0] = c[0][0]
            centroids[label_i,1] = c[0][1]
        return centroids

    def pcl_callback(self, header,x,y,labels):
        # x, y and labels have shape (nsamples,)

        z = np.zeros_like(x)
        points = np.column_stack((x, y, z, labels ))

        dtype = PointField.FLOAT32
        fields = [PointField(name='x',         offset=0,  datatype=dtype, count=1),
                  PointField(name='y',         offset=4,  datatype=dtype, count=1),
                  PointField(name='z',         offset=8,  datatype=dtype, count=1),
                  PointField(name='intensity', offset=12, datatype=dtype, count=1)]             
        pc2_msg = point_cloud2.create_cloud(header, fields, points)
        self.pcl_publisher_.publish(pc2_msg)                

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = ClusterPlot()

    rclpy.spin(minimal_subscriber)
 
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
