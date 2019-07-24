# calibrate

`rosrun camera_calibration cameracalibrator.py --size 7x8 --square 0.09 image:=/rayfin/image_raw camera:=/rayfin --no-service-check`

please check size again (0.09)

# run cage view

`roslaunch syrena_ar_track filter_subsea_cage.launch`


fov_y = radians(54) = 0.9424777960769379


dtheta_y = atan( 2 * dy * tan( fov_y / 2 ) / h )


atan2(2*432.02*tan(0.9424777960769379/2.0) , 1440)
> 0.29670482212040666
r

radians(17) = 0.29670597283903605


+ px vertical 432.02