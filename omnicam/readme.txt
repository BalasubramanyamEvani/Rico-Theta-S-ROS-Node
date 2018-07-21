1) Put calib results obtained from OcamCalib in Results_OcamCalib named as
calib_results_left.txt and calib_results_right.txt for both dual fish eye images.

2) Launch file in omni_cam_launch.

3) run the script 'publisher.py' in scripts folder.

4) Functions used can be seen in ocamFunctions.py

The node publishes the following topic:

 * /omni_cam/image_raw : publishes raw image
 * omni_right/image_raw : raw right frame 
 * omni_left/image_raw : raw left frame 
 * omni_right/rect_image : rectified right image
 * omni_left/rect_image : rectified left image
 * omni_left_info : OcamCalib o/p parameters for left frame 
 * omni_right_info : OcamCalib o/p parameters for right frame
