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
    nh_priv.param("publish_voltage_shunt", param_publish_voltage_shunt, (bool)false);
    nh_priv.param("publish_voltage_bus", param_publish_voltage_bus, (bool)true);
    nh_priv.param("publish_power", param_publish_power, (bool)false);
    nh_priv.param("publish_current", param_publish_current, (bool)true);
    nh_priv.param("frame_id", param_frame_id, (std::string)"imu");
    nh_priv.param("calibration", param_calibration, (int)4096);
    nh_priv.param("rshunt", param_rshunt, (double)0.1);

    current_lsb = 0.04096 / (param_calibration * param_rshunt); // from page 12 of INA219 datasheet
    power_lsb = 20 * current_lsb; // from page 12 of INA219 datasheet
}

// ******** private methods ******** //
bool INA219Activity::reset() {
    int i = 0;

    // __bswap_16 because INA219 is big endian

    _i2c_smbus_write_word_data(file, INA219_REG_CALIBRATION, __bswap_16(param_calibration));
    ros::Duration(0.500).sleep();

    _i2c_smbus_write_word_data(file, INA219_REG_CONFIGURATION, __bswap_16(
            INA219_CONFIG_BVOLTAGERANGE_32V |
            INA219_CONFIG_GAIN_8_320MV |
            INA219_CONFIG_BADCRES_12BIT |
            INA219_CONFIG_SADCRES_12BIT_128S_69MS |
            INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS));
    ros::Duration(0.500).sleep();

    return true;
}

// ******** public methods ******** //

bool INA219Activity::start() {
    ROS_INFO("starting");

    if(!pub_voltage_shunt && param_publish_voltage_shunt)
        pub_voltage_shunt = nh.advertise<std_msgs::Float32>("voltage_shunt", 1);
    if(!pub_voltage_bus && param_publish_voltage_bus)
        pub_voltage_bus = nh.advertise<std_msgs::Float32>("voltage", 1);
    if(!pub_power && param_publish_power)
        pub_power = nh.advertise<std_msgs::Float32>("power", 1);
    if(!pub_current && param_publish_current)
        pub_current = nh.advertise<std_msgs::Float32>("current", 1);

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

    // __bswap_16 because INA219 is big endian

    if(pub_voltage_shunt) {
        std_msgs::Float32 msg_voltage_shunt;
        msg_voltage_shunt.data = 0.00001 * __bswap_16(_i2c_smbus_read_word_data(file, INA219_REG_SHUNT_VOLTAGE));
        pub_voltage_shunt.publish(msg_voltage_shunt);
    }

    if(pub_voltage_bus) {
        std_msgs::Float32 msg_voltage_bus;
        msg_voltage_bus.data = 0.004 * ((int16_t)__bswap_16(_i2c_smbus_read_word_data(file, INA219_REG_BUS_VOLTAGE)) >> 3);
        pub_voltage_bus.publish(msg_voltage_bus);
    }

    if(pub_current) {
        std_msgs::Float32 msg_current;
        msg_current.data = current_lsb * (int16_t)__bswap_16(_i2c_smbus_read_word_data(file, INA219_REG_CURRENT));
        pub_current.publish(msg_current);
    }

    if(pub_power) {
        std_msgs::Float32 msg_power;
        msg_power.data = power_lsb * __bswap_16(_i2c_smbus_read_word_data(file, INA219_REG_POWER));
        pub_power.publish(msg_power);
    }

    return true;    
}

bool INA219Activity::stop() {
    ROS_INFO("stopping");

    if(pub_voltage_shunt) pub_voltage_shunt.shutdown();
    if(pub_voltage_bus) pub_voltage_bus.shutdown();
    if(pub_power) pub_power.shutdown();
    if(pub_current) pub_current.shutdown();

    if(file) close(file);

    return true;
}

}
