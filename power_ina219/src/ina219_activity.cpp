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
    std::string param_name = "/ina219/";
    ROS_INFO("initializing");
    nh_priv.param(param_name + "device", param_device, (std::string)"/dev/i2c-1");
    nh_priv.param(param_name + "address", param_address, (int)INA219_ADDRESS);
    nh_priv.param(param_name + "calibration", param_calibration, (int)4096);
    nh_priv.param(param_name + "rshunt", param_rshunt, (double)0.1);
    nh_priv.param(param_name + "technology", param_battery_technology, (int)sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN);
    nh_priv.param(param_name + "cells", param_cells, (int)0);
    nh_priv.param(param_name + "capacity", param_capacity, (double)nan(""));



    current_lsb = 0.04096 / (param_calibration * param_rshunt); // from page 12 of INA219 datasheet
    power_lsb = 20 * current_lsb; // from page 12 of INA219 datasheet
}

// ******** private methods ******** //
bool INA219Activity::reset() {
    int i = 0;

    // INA219 is big endian
    i2c_smbus_write_word_data(file, INA219_REG_CONFIGURATION, htobe16(CONFIG_RESET));
    ros::Duration(0.10).sleep();

    i2c_smbus_write_word_data(file, INA219_REG_CALIBRATION, htobe16(param_calibration));
    ros::Duration(0.10).sleep();

    i2c_smbus_write_word_data(file, INA219_REG_CONFIGURATION, htobe16(
            INA219_CONFIG_BVOLTAGERANGE_32V |
            INA219_CONFIG_GAIN_8_320MV |
            INA219_CONFIG_BADCRES_12BIT |
            INA219_CONFIG_SADCRES_12BIT_128S_69MS |
            INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS));
    ros::Duration(0.10).sleep();

    return true;
}

// ******** public methods ******** //

bool INA219Activity::start() {
    ROS_INFO("starting");
    pub_battery_state = nh.advertise<sensor_msgs::BatteryState>("battery_state", 1);

    file = open(param_device.c_str(), O_RDWR);
    if(ioctl(file, I2C_SLAVE, param_address) < 0) {
        ROS_ERROR("i2c device open failed");
        return false;
    }

    if(!reset()) {
        ROS_ERROR("chip reset and setup failed");
        return false;
    }
    msg_battery_state.present = true;
    msg_battery_state.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    msg_battery_state.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
    msg_battery_state.power_supply_technology = param_battery_technology;
    msg_battery_state.cell_voltage.resize(param_cells, nan(""));
    return true;
}

bool INA219Activity::spinOnce() {
    ros::spinOnce();

    ros::Time time = ros::Time::now();

    // INA219 is big endian
    msg_battery_state.voltage  = 0.004 * (((uint16_t)be16toh(i2c_smbus_read_word_data(file, INA219_REG_BUS_VOLTAGE))) >> 3);
    msg_battery_state.current = -1.0 * current_lsb * (int16_t)be16toh(i2c_smbus_read_word_data(file, INA219_REG_CURRENT));
    msg_battery_state.capacity = param_capacity < 0 ? nan("") : param_capacity;
    msg_battery_state.design_capacity = msg_battery_state.capacity;
    msg_battery_state.charge = nan("");
    msg_battery_state.percentage = nan("");

    pub_battery_state.publish(msg_battery_state);
    return true;    
}

bool INA219Activity::stop() {
    ROS_INFO("stopping");

    if(pub_battery_state) {
        pub_battery_state.shutdown();
    }

    if(file) {
        close(file);
    }

    return true;
}

}
