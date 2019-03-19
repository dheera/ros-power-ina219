/* power_ina219_i2c_node.cpp
 * Author: Dheera Venkatraman <dheera@dheera.net>
 *
 * Instantiates a INA219 Activity class, as well as
 * a Watchdog that causes this node to die if things aren't
 * working.
 */

<<<<<<< HEAD:power_ina219/src/power_ina219_node.cpp
#include <power_ina219/ina219_activity.h>
=======
#include <pwm_pca9685/pca9685_activity.h>
>>>>>>> 5423c42620c5f128b8c94a3f33da8a5fef6f9f58:pwm_pca9685/src/pwm_pca9685_node.cpp
#include <csignal>

int main(int argc, char *argv[]) {
    ros::NodeHandle* nh = NULL;
    ros::NodeHandle* nh_priv = NULL;

<<<<<<< HEAD:power_ina219/src/power_ina219_node.cpp
    power_ina219::INA219Activity* activity = NULL;
=======
    pwm_pca9685::PCA9685Activity* activity = NULL;
>>>>>>> 5423c42620c5f128b8c94a3f33da8a5fef6f9f58:pwm_pca9685/src/pwm_pca9685_node.cpp

    ros::init(argc, argv, "power_ina219_i2c_node");

    nh = new ros::NodeHandle();
    if(!nh) {
        ROS_FATAL("Failed to initialize NodeHandle");
        ros::shutdown();
        return -1;
    }

    nh_priv = new ros::NodeHandle("~");
    if(!nh_priv) {
        ROS_FATAL("Failed to initialize private NodeHandle");
        delete nh;
        ros::shutdown();
        return -2;
    }

<<<<<<< HEAD:power_ina219/src/power_ina219_node.cpp
    activity = new power_ina219::INA219Activity(*nh, *nh_priv);
=======
    activity = new pwm_pca9685::PCA9685Activity(*nh, *nh_priv);
>>>>>>> 5423c42620c5f128b8c94a3f33da8a5fef6f9f58:pwm_pca9685/src/pwm_pca9685_node.cpp

    if(!activity) {
        ROS_FATAL("Failed to initialize driver");
        delete nh_priv;
        delete nh;
        ros::shutdown();
        return -3;
    }

    if(!activity->start()) {
        ROS_ERROR("Failed to start activity");
        delete nh_priv;
        delete nh;
        ros::shutdown();
        return -4;
    }

    ros::Rate rate(100);
    while(ros::ok()) {
        rate.sleep();
        activity->spinOnce();
    }

    activity->stop();

    delete activity;
    delete nh_priv;
    delete nh;

    return 0;
}
