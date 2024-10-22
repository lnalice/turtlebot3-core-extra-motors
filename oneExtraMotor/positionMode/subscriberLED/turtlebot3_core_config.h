/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho, Gilbert */

#ifndef TURTLEBOT3_CORE_CONFIG_H_
#define TURTLEBOT3_CORE_CONFIG_H_
#define NOETIC_SUPPORT          //uncomment this if writing code for ROS1 Noetic

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h> // third motor
#include <std_msgs/Float64MultiArray.h> //third motor & fourth motor
#include <std_msgs/Byte.h> // 4 LEDs
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <turtlebot3_msgs/SensorState.h>
#include <turtlebot3_msgs/Sound.h>
#include <turtlebot3_msgs/VersionInfo.h>

#include <TurtleBot3.h>
#include "turtlebot3_waffle.h"

#include <math.h>

#define FIRMWARE_VER "1.2.6"

#define CONTROL_MOTOR_SPEED_FREQUENCY          30   //hz
#define CONTROL_MOTOR_TIMEOUT                  500  //ms
#define IMU_PUBLISH_FREQUENCY                  200  //hz
#define CMD_VEL_PUBLISH_FREQUENCY              30   //hz
#define DRIVE_INFORMATION_PUBLISH_FREQUENCY    30   //hz
#define VERSION_INFORMATION_PUBLISH_FREQUENCY  1    //hz 
#define DEBUG_LOG_FREQUENCY                    10   //hz 

#define BLINK_LED_TIMEOUT                      4000  //ms

#define WHEEL_NUM                        2

#define LEFT                             0
#define RIGHT                            1
#define THIRD                            2

#define LINEAR                           0
#define ANGULAR                          1

#define DEG2RAD(x)                       (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                       (x * 57.2957795131)  // *180/PI

#define TICK2RAD                         0.001533981  // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f

#define TEST_DISTANCE                    0.300     // meter
#define TEST_RADIAN                      3.14      // 180 degree

// #define DEBUG                            
#define DEBUG_SERIAL                     SerialBT2

// #define DXL_SERIAL
#define DXL_SERIAL                       Serial3

// Callback function prototypes
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);
void soundCallback(const turtlebot3_msgs::Sound& sound_msg);
void motorPowerCallback(const std_msgs::Bool& power_msg);
void resetCallback(const std_msgs::Empty& reset_msg);
// void moduleVelocityCallback(const std_msgs::Float64& mdl_vel_msg); //third motor - velocity mode
void modulePositionCallback(const std_msgs::Float64MultiArray& mdl_pos_msg); //third motor - Extended Position Control mode
void blinkLedCallback(const std_msgs::Byte& blink_led_msg); // 4 LEDs to indicate direction of movement

// Function prototypes
void publishCmdVelFromRC100Msg(void);
void publishImuMsg(void);
void publishMagMsg(void);
void publishSensorStateMsg(void);
void publishVersionInfoMsg(void);
void publishBatteryStateMsg(void);
void publishDriveInformation(void);

ros::Time rosNow(void);
ros::Time addMicros(ros::Time & t, uint32_t _micros); // deprecated

void updateVariable(bool isConnected);
void updateMotorInfo(int32_t left_tick, int32_t right_tick);
void updateTime(void);
void updateOdometry(void);
void updateJoint(void);
void updateTF(geometry_msgs::TransformStamped& odom_tf);
void updateGyroCali(bool isConnected);
void updateGoalVelocity(void);
void updateTFPrefix(bool isConnected);

void initOdom(void);
void initJointStates(void);

bool calcOdometry(double diff_time);

void sendLogMsg(void);
void waitForSerialLink(bool isConnected);

/*******************************************************************************
* LED
*******************************************************************************/
static bool led_sub = false;

const uint8_t LED_PIN_CNT = 4;
const uint8_t LED_BLINK_TIMES = 2;

const uint8_t LED_FRONT = 50;
const uint8_t LED_RIGHT = 51;
const uint8_t LED_BACK = 52;
const uint8_t LED_LEFT = 54;
uint8_t LED_PIN_LIST[LED_PIN_CNT] = {LED_FRONT, LED_RIGHT, LED_BACK, LED_LEFT};

byte led_status = 0b0000;
bool led_call = false;

void blinkLed(void);

/*******************************************************************************
* Extended Position Mode: SyncWrite
*******************************************************************************/
const float DYNAMIXEL_DEG_PER_PULSE = 0.087890625; // degree : pulse = 360 : 4096(12bits encoder)
const float DYNAMIXEL_PROTOCOL_VERSION = 2.0;
const uint8_t DXL_ID_CNT = 1;
const uint8_t DXL_ID = 4;
const uint16_t user_pkt_buf_cap = 128;
uint8_t user_pkt_buf[user_pkt_buf_cap];

float profile_vel = 0.0;
float dxl_cur_pos = 0.0 ; // DXL Present Position by DXL_ID

// Starting address of the Data to read; Present Position = 132
const uint16_t SR_START_ADDR = 132;
// Length of the Data to read; Length of Position data of X series is 4 byte
const uint16_t SR_ADDR_LEN = 4;
// Starting address of the Data to write; Goal Position = 116
const uint16_t SW_START_ADDR = 116;
// Length of the Data to write; Length of Position data of X series is 4 byte
const uint16_t SW_ADDR_LEN = 4;
typedef struct sr_data{
  int32_t present_position;
} __attribute__((packed)) sr_data_t;
typedef struct sw_data{
  int32_t goal_position;
} __attribute__((packed)) sw_data_t;

sr_data_t sr_data[DXL_ID_CNT];
DYNAMIXEL::InfoSyncReadInst_t sr_infos;
DYNAMIXEL::XELInfoSyncRead_t info_xels_sr[DXL_ID_CNT];

sw_data_t sw_data[DXL_ID_CNT];
DYNAMIXEL::InfoSyncWriteInst_t sw_infos;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw[DXL_ID_CNT];

/*******************************************************************************
* ROS NodeHandle
*******************************************************************************/
ros::NodeHandle nh;
ros::Time current_time;
uint32_t current_offset;

/*******************************************************************************
* ROS Parameter
*******************************************************************************/
char get_prefix[10];
char* get_tf_prefix = get_prefix;

char odom_header_frame_id[30];
char odom_child_frame_id[30];

char imu_frame_id[30];
char mag_frame_id[30];

char joint_state_header_frame_id[30];

/*******************************************************************************
* Subscriber
*******************************************************************************/
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);

ros::Subscriber<turtlebot3_msgs::Sound> sound_sub("sound", soundCallback);

ros::Subscriber<std_msgs::Bool> motor_power_sub("motor_power", motorPowerCallback);

ros::Subscriber<std_msgs::Empty> reset_sub("reset", resetCallback);

// ros::Subscriber<std_msgs::Float64> module_vel_sub("module_vel", moduleVelocityCallback); /* third motor [Velocity Mode] */

ros::Subscriber<std_msgs::Float64MultiArray> module_pos_sub("module_pos", modulePositionCallback); /* third motor [Extended Position Control Mode*/

ros::Subscriber<std_msgs::Byte> blink_led_sub("blink_led", blinkLedCallback); 
/* LED [4 LEDs to indicate direction of movement] */

/*******************************************************************************
* Publisher
*******************************************************************************/
// Bumpers, cliffs, buttons, encoders, battery of Turtlebot3
turtlebot3_msgs::SensorState sensor_state_msg;
ros::Publisher sensor_state_pub("sensor_state", &sensor_state_msg);

// Version information of Turtlebot3
turtlebot3_msgs::VersionInfo version_info_msg;
ros::Publisher version_info_pub("firmware_version", &version_info_msg);

// IMU of Turtlebot3
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

// Command velocity of Turtlebot3 using RC100 remote controller
geometry_msgs::Twist cmd_vel_rc100_msg;
ros::Publisher cmd_vel_rc100_pub("cmd_vel_rc100", &cmd_vel_rc100_msg);

// Odometry of Turtlebot3
nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);

// Joint(Dynamixel) state of Turtlebot3
sensor_msgs::JointState joint_states;
ros::Publisher joint_states_pub("joint_states", &joint_states);

// Battey state of Turtlebot3
#if defined NOETIC_SUPPORT
sensor_msgs::BatteryStateNoetic battery_state_msg;
#else
sensor_msgs::BatteryState battery_state_msg;
#endif
ros::Publisher battery_state_pub("battery_state", &battery_state_msg);

// Magnetic field
sensor_msgs::MagneticField mag_msg;
ros::Publisher mag_pub("magnetic_field", &mag_msg);

/*******************************************************************************
* Transform Broadcaster
*******************************************************************************/
// TF of Turtlebot3
geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tf_broadcaster;

/*******************************************************************************
* SoftwareTimer of Turtlebot3
*******************************************************************************/
static uint32_t tTime[10];

/*******************************************************************************
* Declaration for motor
*******************************************************************************/
Turtlebot3MotorDriver motor_driver;

/*******************************************************************************
* Calculation for odometry
*******************************************************************************/
bool init_encoder = true;
int32_t last_diff_tick[WHEEL_NUM] = {0, 0};
double  last_rad[WHEEL_NUM]       = {0.0, 0.0};

/*******************************************************************************
* Update Joint State
*******************************************************************************/
double  last_velocity[WHEEL_NUM]  = {0.0, 0.0};

/*******************************************************************************
* Declaration for sensors
*******************************************************************************/
Turtlebot3Sensor sensors;

/*******************************************************************************
* Declaration for controllers
*******************************************************************************/
Turtlebot3Controller controllers;
float zero_velocity[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_button[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_cmd[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_rc100[WHEEL_NUM] = {0.0, 0.0};
// float module_velocity = 0.0; /* third motor */

/*******************************************************************************
* Declaration for diagnosis
*******************************************************************************/
Turtlebot3Diagnosis diagnosis;

/*******************************************************************************
* Declaration for SLAM and navigation
*******************************************************************************/
unsigned long prev_update_time;
float odom_pose[3];
double odom_vel[3];

/*******************************************************************************
* Declaration for Battery
*******************************************************************************/
bool setup_end        = false;
uint8_t battery_state = 0;

#endif // TURTLEBOT3_CORE_CONFIG_H_