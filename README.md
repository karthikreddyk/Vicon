###<center>[Vicon Listener (ROScpp)](https://github.com/lakehanne/Vicon)</center>

Based on [Markus Achtelik's](https://github.com/markusachtelik) [ROS bridge](https://github.com/ethz-asl/vicon_bridge) to the Vicon [Data Stream SDK](http://www.vicon.com/products/software/datastream-sdk), I have provided a ROS Metapackage system that computes the rotation and translation coordinates of Vicon **mocap** segment markers (using the numerically stable Gram-Schmidt orthonormalization procedure). I have provided an example for markers on a manikin head after subscribing to the published topic "vicon/markers" from the ros bridge to vicon sdk. To run , simply do

<pre class="terminal"><code>rosrun vicon_listener vicon_listener</code></pre>

The package also provides the `ServiceClient` to the advertised pose `ServiceServer` consisting of translation and orientation quarternions and can be run with 

<pre class="terminal"><code>rosrun vicon_bridge grabpose "subject_name" "segment_name" grab_vicon_pose</code></pre>

It's been tested and verified on ROS_Indigo running Ubuntu Trusty. If you can verify that it works on Jade or Hydro, kindly leave a feedback on the [project's issues page](https://github.com/lakehanne/vicon/issues). 
