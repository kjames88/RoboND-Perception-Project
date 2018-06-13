#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml
import pcl

WORLD = 3

# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = '{}'.format(object_name.data) # crashes with type error otherwise
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)
    
    #  Statistical Outlier Filtering
    #   http://www.pointclouds.org/news/2013/02/07/python-bindings-for-the-point-cloud-library/
    fil = cloud.make_statistical_outlier_filter()
    fil.set_mean_k (1)
    fil.set_std_dev_mul_thresh (0.25)
    cloud = fil.filter()

    # Voxel Grid Downsampling
    vox = cloud.make_voxel_grid_filter()
    LEAF_SIZE = 0.01
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()

    # PassThrough Filter
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.6
    axis_max = 1.0
    passthrough.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough.filter()
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'y'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = -0.5
    axis_max = 0.5
    passthrough.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough.filter()


    # RANSAC Plane Segmentation
    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)
    inliers, coefficients = seg.segment()

    # Extract inliers and outliers
    #   Note this is extraction negation so the limits are such that
    #   between axis_min and axis_max ends up in cloud_objects, not the reverse
    cloud_table = cloud_filtered.extract(inliers, negative=False)
    cloud_objects = cloud_filtered.extract(inliers, negative=True)

    # Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()

    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold 
    # as well as minimum and maximum cluster size (in points)
    ec.set_ClusterTolerance(0.015)
    ec.set_MinClusterSize(50)
    ec.set_MaxClusterSize(2000)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()

    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []
    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],white_cloud[indice][1],white_cloud[indice][2],
                                            rgb_to_float(cluster_color[j])])
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # Convert PCL data to ROS messages
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)

# Exercise-3 TODOs:

    detected_objects_labels = []
    detected_objects_list = []

    # Classify the clusters! (loop through each detected cluster one at a time)

    for index, pts_list in enumerate(cluster_indices):

        # Grab the points for the cluster

        # Grab the points for the cluster from the extracted outliers (cloud_objects)
        pcl_cluster = cloud_objects.extract(pts_list)
        # convert the cluster from pcl to ROS using helper function
        ros_cluster = pcl_to_ros(pcl_cluster)

        # Compute the associated feature vector

        # Extract histogram features
        #   complete this step just as is covered in capture_features.py
        # Extract histogram features
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))

        # Make the prediction

        # Make the prediction, retrieve the label for the result
        # and add it to detected_objects_labels list
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects_list.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects

    if True:
        try:
            pr2_mover(detected_objects_list)
        except rospy.ROSInterruptException:
            pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):


    # Initialize variables
    labels = []
    centroids = []
    yaml_dicts = []

    # Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    dropbox_param = rospy.get_param('/dropbox')

    # Parse parameters into individual variables
    #     object_name = object_list_param[i]['name']
    #     object_group = object_list_param[i]['group']
    for object in object_list:
        labels.append(object.label)
        object_arr = ros_to_pcl(object.cloud).to_array()
        centroids.append(np.mean(object_arr, axis=0)[:3])

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # Loop through the pick list

    print('WORLD = {} size of pick list is {} size of detected list is {}'.format(WORLD,len(object_list_param),len(object_list)))

    for obj in object_list_param:
        obj_name = obj['name']
        obj_group = obj['group']

        # Get the PointCloud for a given object and obtain its centroid

        print('Operation:  move object {}'.format(obj_name))

        # find matching name in labels
        sel_obj = -1
        for i in range(len(labels)):
            if labels[i] == obj_name:
                print('found object {} with name {} and centroid {}'.format(i,obj_name,centroids[i]))
                sel_obj = i
                # crosscheck using arm in case of bad classification
                if (centroids[i][1] > -0.05 and obj_group == 'red') or (centroids[i][1] < 0.05 and obj_group == 'green'):
                    break
                else:
                    print('suspicious group vs centroid position - continue looking')

        if sel_obj < 0:
            print('no matching object with name {} found - skipping', obj_name)
            continue

        # Create 'place_pose' for the object
        place_pose = Pose()

        # Assign the arm to be used for pick_place
        arm = 'left'
        if obj_group == 'green':
            arm = 'right'
        print('arm for operation {} group {}'.format(arm,obj_group))

        target_pose = []
        for i in range(len(dropbox_param)):
            if dropbox_param[i]['name'] == arm:
                target_pose = dropbox_param[i]['position']
                break

        print('dropbox position is {}'.format(target_pose))
        place_pose.position.x = float(target_pose[0])
        place_pose.position.y = float(target_pose[1])
        place_pose.position.z = float(target_pose[2])

        # Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        test_scene_num = Int32()
        arm_name = String()
        _object_name = String()
        pick_pose = Pose()

        test_scene_num.data = WORLD
        arm_name.data = arm
        _object_name.data = labels[sel_obj]

        pick_pose.position.x = float(centroids[sel_obj][0])
        pick_pose.position.y = float(centroids[sel_obj][1])
        pick_pose.position.z = float(centroids[sel_obj][2])

        d = make_yaml_dict(test_scene_num, arm_name, _object_name, pick_pose, place_pose)

        yaml_dicts.append(d)

        if False:
            # Wait for 'pick_place_routine' service to come up
            rospy.wait_for_service('pick_place_routine')

            try:
                pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

                # Insert your message variables to be sent as a service request
                resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)

                print ("Response: ",resp.success)

            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

    print ('dict len is {}'.format(len(yaml_dicts)))
    # Output your request parameters into output yaml file

    send_to_yaml('world{}.yaml'.format(WORLD), yaml_dicts)



if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    # Create Subscribers
    pcl_sub = rospy.Subscriber('/pr2/world/points', pc2.PointCloud2, pcl_callback, queue_size=1)

    # Create Publishers
    pcl_objects_pub = rospy.Publisher('/pcl_objects', PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher('/pcl_table', PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher('/pcl_cluster', PointCloud2, queue_size=1)

    object_markers_pub = rospy.Publisher('/object_markers', Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher('/detected_objects', DetectedObjectsArray, queue_size=1)

    # Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']

    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    print('model classes:  {}'.format(model['classes']))

    # Initialize color_list
    get_color_list.color_list = []

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
