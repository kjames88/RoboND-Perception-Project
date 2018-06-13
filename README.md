## Project: Perception Pick & Place

### Object Recognition Classifier

A single classifier was trained on 8 object types:  *sticky_notes*, *book*, *snacks*, *biscuits*, *eraser*, *soap*, *soap2*, and *glue*.  Representative point clouds were captured using the supplied training.launch and capture_features.py.  I used 100 iterations of capture per type, with 32 bins for HSV and 32 bins for surface normals.  The classifier uses the *linear* SVC kernel.  The resulting confusion matrix is shown below.

![confusion] (images/screenshot_confusion.png "Confusion Matrices")

#### Difficulties

Most of the time on this project went into training classifiers and then failing requirements on one or more the worlds.  In general, the confusion matrix scores, where accuracy score was 0.78 or higher, had virtually no correlation with ability to pass the test cases.  *rbf* kernel in particular produced a somewhat better accuracy score but never classified anything as other than *book* in use.  More importantly, there seems to be a race condition in the capture process.  Thanks to *Jamie Cho* on the slack channel for pointing out the problem.  For me, adding 100ms delay in training_helper.py before capturing a point cloud message made a significant difference, although as before the most profound improvement was in the accuracy score.  Note that this was running native linux and native ROS.

### ROS Node

#### Point Cloud Filtering

Most of the code for subscribing to /pr2/world/points was reused from the lesson exercises.  Prior to those steps, I applied a statistical outlier filter (http://www.pointclouds.org/news/2013/02/07/python-bindings-for-the-point-cloud-library/).  After tuning this filter, most of the noise was eliminated.  Following this, voxel grid downsampling and passthrough filtering on both z and y axes limit the processed voxels to objects on the table and not including the drop boxes.

RANSAC plane filter separates the table from the objects of interest, and Euclidean clustering groups voxels into object clusters.  After this, HSV color and surface normal histograms are generated for the object clusters.  The classifier then generates labels based on the histogram inputs.


![raw] (images/screenshot_rawpoints.png "Input world point cloud")


![objects] (images/screenshot_objects.png "Point cloud after RANSAC")


![clusters] (images/screenshot_clusters.png "Object clusters")


#### ROS Message Generation

The detected objects list resulting from the above process is then used to generate ROS messages that could be used to move the robot arms and deliver the selected objects to the drop boxes.  The first step is to iterate over the pick list of objects for the test world (supplied by *object_list_param*), and find the matching detected object for each.  This is simple if the recognition has produced unique labels, but in case there were duplicates I used a simple filter assuming the object location should be near the side of the target drop box to try to optimize the selection.  This is a blunt tool that still requires nearly correct classification.

The identified predicted object for each pick object then has its centroid computed and this becomes the *pick_pose*.  The group association with the pick object determines the arm to be used (red: *left*, green: *right*), and this further determines the drop box location and *place_pose*.  *dropbox_param* supplies the place poses, but *test_scene_num* is hard coded for each test world.

Once these fields are determined, a ROS service call should move the selected arm, but when I activated this code the simulation hung.

#### YAML Output

The primary product of this project is the YAML file containing the information used by the ROS service (*test_scene_num*, *arm_name*, *object_name*, *pick_pose*, *place_pose*) for each object.  The resulting yaml files are contained in submit/worldX.yaml.

In the case of world 2, *book* was incoreectly classified as *snacks* resulting in a missing entry in the yaml file.  The other two worlds are complete.

### Results

![world1] (images/screenshot_world1.png "World 1 recognition")

![world2] (images/screenshot_world2.png "World 2 recognition")

![world3] (images/screenshot_world3.png "world 3 recognition")










