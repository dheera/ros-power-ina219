/* ina219_activity.cpp
 * Author: Dheera Venkatraman <dheera@dheera.net>
 *
 * Defines a INA219 Activity class, constructed with node handles
 * and which handles all ROS duties.
 */

#include "power_ina219/ina219_activity.h"

namespace power_ina219 {

// ******** constructors ******** //

INA219Activity::INA219Activity(ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv) :
  nh(_nh), nh_priv(_nh_priv) {
    ROS_INFO("initializing");
    nh_priv.param("device", param_device, (std::string)"/dev/i2c-1");
    nh_priv.param("address", param_address, (int)INA219_ADDRESS);
    nh_priv.param("rate", param_rate, (int)10);
    nh_priv.param("publish_shunt_voltage", param_publish_bus_voltage, (bool)false);
    nh_priv.param("publish_bus_voltage", param_publish_bus_voltage, (bool)true);
    nh_priv.param("publish_power", param_publish_bus_voltage, (bool)false);
    nh_priv.param("publish_current", param_publish_bus_voltage, (bool)true);
    nh_priv.param("frame_id", param_frame_id, (std::string)"imu");
    nh_priv.param("calibration", param_calibration, (int)4096);
    nh_priv.param("rshunt", param_rshunt, (double)0.1);

    current_lsb = 0.04096 / (param_calibration * param_rshunt); // from page 12 of INA219 datasheet
    power_lsb = 20 * current_lsb; // from page 12 of INA219 datasheet
}

// ******** private methods ******** //
bool INA219Activity::reset() {
    int i = 0;

    _i2c_smbus_write_word_data(file, INA219_REG_CALIBRATION, (uint16_t)param_calibration);
    ros::Duration(0.500).sleep();

    _i2c_smbus_write_word_data(file, INA219_REG_CONFIGURATION, 
            INA219_CONFIG_BVOLTAGERANGE_32V |
            INA219_CONFIG_GAIN_8_320MV |
            INA219_CONFIG_BADCRES_12BIT |
            INA219_CONFIG_SADCRES_12BIT_1S_532US |
            INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS);
    ros::Duration(0.500).sleep();

    return true;
}

// ******** public methods ******** //

bool INA219Activity::start() {
    ROS_INFO("starting");

    if(!pub_shunt_voltage && param_publish_shunt_voltage)
        pub_shunt_voltage = nh.advertise<std_msgs::Float32>("pub_shunt_voltage", 1);
    if(!pub_bus_voltage && param_publish_bus_voltage)
        pub_bus_voltage = nh.advertise<std_msgs::Float32>("pub_bus_voltage", 1);
    if(!pub_power && param_publish_power)
        pub_power = nh.advertise<std_msgs::Float32>("pub_power", 1);
    if(!pub_current && param_publish_current)
        pub_current = nh.advertise<std_msgs::Float32>("pub_current", 1);

    file = open(param_device.c_str(), O_RDWR);
    if(ioctl(file, I2C_SLAVE, param_address) < 0) {
        ROS_ERROR("i2c device open failed");
        return false;
    }

    if(!reset()) {
        ROS_ERROR("chip reset and setup failed");
        return false;
    }

    return true;
}

bool INA219Activity::spinOnce() {
    ros::spinOnce();

    ros::Time time = ros::Time::now();

    if(pub_shunt_voltage) {
        std_msgs::Float32 msg_shunt_voltage;
        msg_shunt_voltage.data = (double)_i2c_smbus_read_word_data(file, INA219_REG_SHUNT_VOLTAGE) * 0.00001;
        pub_shunt_voltage.publish(msg_shunt_voltage);
    }

    if(pub_shunt_voltage) {
        std_msgs::Float32 msg_bus_voltage;
        msg_bus_voltage.data = (double)(_i2c_smbus_read_word_data(file, INA219_REG_BUS_VOLTAGE) >> 3) * 0.001;
        pub_shunt_voltage.publish(msg_bus_voltage);
    }

    if(pub_current) {
        std_msgs::Float32 msg_current;
        msg_current.data = (double)_i2c_smbus_read_word_data(file, INA219_REG_CURRENT) * current_lsb;
        pub_shunt_voltage.publish(msg_current);
    }

    if(pub_power) {
        std_msgs::Float32 msg_power;
        msg_power.data = (double)_i2c_smbus_read_word_data(file, INA219_REG_POWER) * power_lsb;
        pub_shunt_voltage.publish(msg_power);
    }

    return true;    
}

bool INA219Activity::stop() {
    ROS_INFO("stopping");

    if(pub_shunt_voltage) pub_shunt_voltage.shutdown();
    if(pub_bus_voltage) pub_bus_voltage.shutdown();
    if(pub_power) pub_power.shutdown();
    if(pub_current) pub_current.shutdown();

    if(file) close(file);

    return true;
}

}
