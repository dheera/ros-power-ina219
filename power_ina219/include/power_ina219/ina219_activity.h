#ifndef _ina219_activity_dot_h
#define _ina219_activity_dot_h

#include <ros/ros.h>
#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <endian.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
extern "C"{
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}

#define INA219_ADDRESS                         0x40

#define INA219_REG_CONFIGURATION               0x00
#define INA219_REG_SHUNT_VOLTAGE               0x01
#define INA219_REG_BUS_VOLTAGE                 0x02
#define INA219_REG_POWER                       0x03
#define INA219_REG_CURRENT                     0x04
#define INA219_REG_CALIBRATION                 0x05

#define CONFIG_RESET                           0x8000 // Reset Bit

#define INA219_CONFIG_BVOLTAGERANGE_MASK       0x2000 // Bus Voltage Range Mask
#define INA219_CONFIG_BVOLTAGERANGE_16V        0x0000 // 0-16V Range
#define INA219_CONFIG_BVOLTAGERANGE_32V        0x2000 // 0-32V Range

#define INA219_CONFIG_GAIN_MASK                0x1800 // Gain Mask
#define INA219_CONFIG_GAIN_1_40MV              0x0000 // Gain 1, 40mV Range
#define INA219_CONFIG_GAIN_2_80MV              0x0800 // Gain 2, 80mV Range
#define INA219_CONFIG_GAIN_4_160MV             0x1000 // Gain 4, 160mV Range
#define INA219_CONFIG_GAIN_8_320MV             0x1800 // Gain 8, 320mV Range

#define INA219_CONFIG_BADCRES_MASK             0x0780 // Bus ADC Resolution Mask
#define INA219_CONFIG_BADCRES_9BIT             0x0080 // 9-bit bus res = 0..511
#define INA219_CONFIG_BADCRES_10BIT            0x0100 // 10-bit bus res = 0..1023
#define INA219_CONFIG_BADCRES_11BIT            0x0200 // 11-bit bus res = 0..2047
#define INA219_CONFIG_BADCRES_12BIT            0x0400 // 12-bit bus res = 0..4097

#define INA219_CONFIG_SADCRES_MASK             0x0078 // Shunt ADC Resolution and Averaging Mask
#define INA219_CONFIG_SADCRES_9BIT_1S_84US     0x0000 // 1 x 9-bit shunt sample
#define INA219_CONFIG_SADCRES_10BIT_1S_148US   0x0008 // 1 x 10-bit shunt sample
#define INA219_CONFIG_SADCRES_11BIT_1S_276US   0x0010 // 1 x 11-bit shunt sample
#define INA219_CONFIG_SADCRES_12BIT_1S_532US   0x0018 // 1 x 12-bit shunt sample
#define INA219_CONFIG_SADCRES_12BIT_2S_1060US  0x0048 // 2 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_4S_2130US  0x0050 // 4 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_8S_4260US  0x0058 // 8 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_16S_8510US 0x0060 // 16 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_32S_17MS   0x0068 // 32 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_64S_34MS   0x0070 // 64 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_128S_69MS  0x0078 // 128 x 12-bit shunt samples averaged together

#define INA219_CONFIG_MODE_MASK                0x0007 // Operating Mode Mask
#define INA219_CONFIG_MODE_POWERDOWN           0x0000
#define INA219_CONFIG_MODE_SVOLT_TRIGGERED     0x0001
#define INA219_CONFIG_MODE_BVOLT_TRIGGERED     0x0002
#define INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED 0x0003
#define INA219_CONFIG_MODE_ADCOFF              0x0004
#define INA219_CONFIG_MODE_SVOLT_CONTINUOUS    0x0005
#define INA219_CONFIG_MODE_BVOLT_CONTINUOUS    0x0006
#define INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS 0x0007


namespace power_ina219 {

class INA219Activity {
  public:
    INA219Activity(ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv);

    bool start();
    bool stop();
    bool spinOnce();

  private:
    bool reset();
    
    // class variables
    uint32_t seq = 0;
    int file;
    double current_lsb;
    double power_lsb;

    // ROS node handles
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv;

    // ROS parameters
    std::string param_frame_id;
    std::string param_device;
    int param_address;
    int param_rate;
    int param_calibration;
    double param_rshunt;
    bool param_publish_voltage_bus;
    bool param_publish_voltage_shunt;
    bool param_publish_power;
    bool param_publish_current;

    // ROS publishers

    // ROS subscribers
    ros::Publisher pub_voltage_bus;
    ros::Publisher pub_voltage_shunt;
    ros::Publisher pub_power;
    ros::Publisher pub_current;

    // ROS services
};

}

#endif // _ina219_activity_dot_h
