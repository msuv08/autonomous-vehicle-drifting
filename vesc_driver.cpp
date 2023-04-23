// -*- mode:c++; fill-column: 100; -*-

#include "vesc_driver.h"

#include <atomic>
#include <cassert>
#include <cmath>
#include <sstream>

#include "boost/bind.hpp"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "ut_automata/CarStatusMsg.h"
#include "ut_automata/VescStateStamped.h"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TwistStamped.h"

#include "config_reader/config_reader.h"
#include "shared/math/math_util.h"
#include <torch/script.h>
#include "iostream"

static const bool kDebug = true;
static const float kCommandRate = 20;
static const float kCommandInterval = 1.0 / kCommandRate;
torch::jit::script::Module module; 

CONFIG_FLOAT(speed_to_erpm_gain_, "speed_to_erpm_gain");
CONFIG_FLOAT(speed_to_erpm_offset_, "speed_to_erpm_offset");
CONFIG_FLOAT(steering_to_servo_gain_, "steering_angle_to_servo_gain");
CONFIG_FLOAT(steering_to_servo_offset_, "steering_angle_to_servo_offset");
CONFIG_FLOAT(wheelbase_, "wheelbase");
CONFIG_FLOAT(erpm_speed_limit_, "erpm_speed_limit");
CONFIG_FLOAT(servo_min_, "servo_min");
CONFIG_FLOAT(servo_max_, "servo_max");
CONFIG_FLOAT(max_accel_, "max_acceleration");
CONFIG_FLOAT(max_decel_, "max_deceleration");
CONFIG_FLOAT(turbo_speed_, "joystick_turbo_speed");
CONFIG_FLOAT(normal_speed_, "joystick_normal_speed");
CONFIG_STRING(serial_port_, "serial_port");

DEFINE_string(config_dir, "config", 
    "Directory containing the car.lua and vesc.lua config files.");

using ut_automata::CarStatusMsg;
using ut_automata::VescStateStamped;

namespace {

float mux_drive_speed_ = 0;
float mux_steering_angle_ = 0;


VescStateStamped state_msg_;
CarStatusMsg car_status_msg_;
}  // namespace

namespace vesc_driver
{

VescDriver::VescDriver(ros::NodeHandle nh,
                       ros::NodeHandle private_nh) :
    vesc_(std::string(),
          boost::bind(&VescDriver::vescPacketCallback, this, _1)),
    driver_mode_(MODE_INITIALIZING),
    drive_mode_(kStoppedDrive),
    fw_version_major_(-1),
    fw_version_minor_(-1),
    t_last_command_(0),
    t_last_joystick_(0) {
  {
    // Load config.
    config_reader::ConfigReader reader({
      FLAGS_config_dir + "/car.lua",
      FLAGS_config_dir + "/vesc.lua"
    });
  }
  state_msg_.header.seq = 0;
  state_msg_.header.frame_id = "base_link";
  car_status_msg_.header = state_msg_.header;

  odom_msg_.header.stamp = ros::Time::now();
  odom_msg_.header.frame_id = "odom";
  odom_msg_.child_frame_id = "base_link";

  odom_msg_.twist.twist.linear.x = 0;
  odom_msg_.twist.twist.linear.y = 0;
  odom_msg_.twist.twist.linear.z = 0;
  odom_msg_.twist.twist.angular.x = 0;
  odom_msg_.twist.twist.angular.y = 0;
  odom_msg_.twist.twist.angular.z = 0;
  odom_msg_.twist.covariance =
                {0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.03};

  odom_msg_.pose.pose.position.x = 0;
  odom_msg_.pose.pose.position.y = 0;
  odom_msg_.pose.pose.position.z = 0;
  odom_msg_.pose.covariance =
                {0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 1000000.0 , 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.03};
  odom_msg_.pose.pose.orientation.w = 1;
  odom_msg_.pose.pose.orientation.x = 0;
  odom_msg_.pose.pose.orientation.y = 0;
  odom_msg_.pose.pose.orientation.z = 0;

  // std::cout << "Model loading started\n";
  // MODEL LOADING HERE
  try {
    module = torch::jit::load("/home/amrl_user/traced_ikd_model.pt");
  }
  catch (const c10::Error& e) {
    std::cerr << "Error loading the model: " << e.what() << std::endl;
    // Handle error appropriately
    // sys.exit(-1)
  }
  // std::cout << "Model loading finished\n";

  // attempt to connect to the serial port
  if (kDebug) printf("CONNECT\n");
  CHECK(vesc_.connect(serial_port_)) << "Failed to connect to the VESC";
  if (kDebug) printf("CONNECTED\n");

  state_pub_ = nh.advertise<VescStateStamped>("sensors/core", 1);
  autonomy_enabler_pub_ = nh.advertise<std_msgs::Bool>("autonomy_enabler", 1);
  odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 1);
  drive_pub_ = nh.advertise<geometry_msgs::TwistStamped>("vesc_drive", 1);
  car_status_pub_ = nh.advertise<CarStatusMsg>("car_status", 1);

  ackermann_curvature_sub_ = nh.subscribe(
      "/ackermann_curvature_drive",
      10,
      &VescDriver::ackermannCurvatureCallback,
      this);
  joystick_sub_ = nh.subscribe(
      "joystick", 10, &VescDriver::joystickCallback, this);

  if (kDebug) printf("TIMER START\n");
  // create a 50Hz timer, used for state machine & polling VESC telemetry
  timer_ = nh.createSteadyTimer(ros::WallDuration(kCommandInterval),
                                &VescDriver::timerCallback,
                                this);
  if (kDebug) printf("DONE INIT\n");
}

void VescDriver::checkCommandTimeout() {
  static const double kTimeout = 0.5;
  const double t_now = ros::WallTime::now().toSec();
  if ((t_now > t_last_command_ + kTimeout && isAutonomous()) ||
      t_now > t_last_joystick_ + kTimeout) {
    mux_drive_speed_ = 0;
    mux_steering_angle_ = 0;
  }
}

geometry_msgs::TwistStamped createTwist(float velocity, float curvature) {
  geometry_msgs::TwistStamped  twist_msg;
  twist_msg.header.stamp = ros::Time::now();
  std::cout << "-- \n";
  std::cout << velocity << std::endl;
  std::cout << curvature << std::endl;
  twist_msg.twist.linear.x = velocity;
  twist_msg.twist.linear.y = 0;
  twist_msg.twist.linear.z = 0;
  twist_msg.twist.angular.x = 0;
  twist_msg.twist.angular.y = 0;
  twist_msg.twist.angular.z = velocity * curvature;
  return twist_msg;
}

geometry_msgs::TwistStamped CalculateDriveCmd(float speed, float steering_angle) {
  float velocity = speed;
  float turn_radius = steering_angle != 0 ? wheelbase_ / tan(steering_angle) : 0;
  float curvature = turn_radius != 0 ?  1.0 / turn_radius : 0;
  // use a circular linked list of csv of teleoperation of only joystick data

  // Create input tensorz
  // Create angular velocity
  // omega/v = curvature
  // omega = v * r (curvature is 1/r)
  // std::cout << "Starting NN test\n";
  // NN ARCHITECTURE WRONG
  float angular_velocity = velocity * curvature;
  float norm_linear_velocity = velocity / 6;
  float norm_ang_velocity = angular_velocity / 4;
  std::vector<float> input = {norm_linear_velocity, norm_ang_velocity};
  at::Tensor tensor = torch::from_blob(input.data(), {1, 2}, torch::kFloat32);
  std::vector<torch::jit::IValue> inputs;
  inputs.push_back(tensor);
  // std::cout << tensor << std::endl;
  // Run the model
  at::Tensor output = module.forward(inputs).toTensor();
  // std::cout << output << std::endl;
  float out = *output[0][0].data_ptr<float>();
  // std::cout << out << std::endl;
  float corr_curvature;
  if (velocity != 0) {
    corr_curvature = out / velocity;
  } else {
    corr_curvature = 0;
  }
  return createTwist(-1 * velocity, curvature);
}

void VescDriver::joystickCallback(const sensor_msgs::Joy& msg) {
  static const bool kDebug = true;
  static const float kMaxTurnRate = 0.25;
  static const float kAxesEps = 0.2;
  static const size_t kManualDriveButton = 4;
  static const size_t kAutonomousDriveButton = 5;
  static const size_t kAutonomousDriveToggleButton = 7;
  if (msg.buttons.size() < 6) return;
  t_last_joystick_ = ros::WallTime::now().toSec();
  int toggle = toggleState(msg.buttons[kAutonomousDriveToggleButton]);

  // determine if any button/axes is pressed.
  bool pressed = (drive_mode_ == kAutonomousContinuousDrive 
    && toggle == kToggleOn);
  for(size_t i = 0; i < msg.buttons.size(); i++){
    if(i != kAutonomousDriveToggleButton && msg.buttons[i]){
      pressed = true;
    }
  }
  for(size_t i = 0; i < msg.axes.size(); i++){
    float axes_value = msg.axes[i];
    if(i == 2 || i == 5){
      // axes 2, 5's initial value is around -1.0
      axes_value += 1.0;
    }
    if(std::abs(axes_value) > kAxesEps){
      pressed = true;    
    }
  }
  if (drive_mode_ == kAutonomousContinuousDrive && pressed){
    // stop the car if any button is pressed
    drive_mode_ = kStoppedDrive;
    mux_drive_speed_ = 0;
    mux_steering_angle_ = 0;
  } else if (msg.buttons[kManualDriveButton] == 1) {
    // joystick mode
    if(kDebug) printf("Joystick\n");
    drive_mode_ = kJoystickDrive;
  } else if ((toggle == kToggleOn) ||
    (drive_mode_ == kAutonomousContinuousDrive && 
    toggle != kToggleOn)){
    if(kDebug) printf("ContAutonomous\n");
    drive_mode_ = kAutonomousContinuousDrive;
  } else if (msg.buttons[kAutonomousDriveButton] == 1) {
    if(kDebug) printf("Autonomous\n");
    drive_mode_ = kAutonomousDrive;
  } else {
    if(kDebug) printf("Stopped\n");
    drive_mode_ = kStoppedDrive;
    mux_drive_speed_ = 0;
    mux_steering_angle_ = 0;
  }
  if (drive_mode_ == kJoystickDrive) {
    if (msg.axes.size() < 5) return;
    const float steer_joystick = -msg.axes[0];
    const float drive_joystick = -msg.axes[4];
    const bool turbo_mode = (msg.axes[2] >= 0.9);
    const float max_speed = (turbo_mode ? turbo_speed_ : normal_speed_);
    float speed = drive_joystick * max_speed;
    float steering_angle = steer_joystick * kMaxTurnRate;
    mux_drive_speed_ = speed;
    mux_steering_angle_ = steering_angle;
    if (kDebug) printf("%7.2f %.1f\u00b0\n", speed, math_util::RadToDeg(steering_angle));
  }

  if (drive_mode_ == kAutonomousDrive || 
      drive_mode_ == kAutonomousContinuousDrive) {
    std_msgs::Bool msg;
    msg.data = true;
    autonomy_enabler_pub_.publish(msg);
  } else {
    std_msgs::Bool msg;
    msg.data = false;
    autonomy_enabler_pub_.publish(msg);
  }
}

  /* TODO or TO-THINKABOUT LIST
    - what should we do on startup? send brake or zero command?
    - what to do if the vesc interface gives an error?
    - check version number against know compatable?
    - should we wait until we receive telemetry before sending commands?
    - should we track the last motor command
    - what to do if no motor command received recently?
    - what to do if no servo command received recently?
    - what is the motor safe off state (0 current?)
    - what to do if a command parameter is out of range, ignore?
    - try to predict vesc bounds (from vesc config) and command detect bounds
errors
  */

float Clip(float x, float x_min, float x_max, const char* name) {
  if (x < x_min) {
    fprintf(stderr,
            "Clipping %s value %f to min limit, %f\n",
            name,
            x,
            x_min);
    return x_min;
  }
  if (x > x_max) {
    fprintf(stderr,
            "Clipping %s value %f to max limit, %f\n",
            name,
            x,
            x_max);
    return x_max;
  }
  return x;
}

void VescDriver::sendDriveCommands() {
  static const bool kDebug = true;
  static float last_speed_ = 0;

  using math_util::Bound;
  const float max_accel =
    ((last_speed_ > 0.0) ? max_accel_ : max_decel_);
  const float max_decel =
    ((last_speed_ > 0.0) ? max_decel_ : max_accel_);
  const float smooth_speed = math_util::Clamp<float>(
      mux_drive_speed_,
      last_speed_ - kCommandInterval * max_decel,
      last_speed_ + kCommandInterval * max_accel);
  last_speed_ = smooth_speed;
  if (kDebug) {
    printf("%7.2f %7.2f %.1f\u00b0\n",
           mux_drive_speed_, smooth_speed, mux_steering_angle_);
  }
  const float erpm =
      speed_to_erpm_gain_ * smooth_speed + speed_to_erpm_offset_;

  // calc steering angle (servo)
  const float servo = steering_to_servo_gain_ * mux_steering_angle_ +
      steering_to_servo_offset_;

  // Set speed command.
  const float erpm_clipped = Clip(erpm, -erpm_speed_limit_, erpm_speed_limit_, "erpm");
  vesc_.setSpeed(erpm_clipped);

  // Set servo position command.
  const float clipped_servo = Clip(servo, servo_min_, servo_max_, "servo");
  vesc_.setServo(clipped_servo);
  mux_steering_angle_ = (clipped_servo - steering_to_servo_offset_) 
                        / steering_to_servo_gain_;
  last_steering_angle_ = mux_steering_angle_;

  const float clipped_speed = (erpm_clipped - speed_to_erpm_offset_) / speed_to_erpm_gain_;
  drive_pub_.publish(CalculateDriveCmd(clipped_speed, mux_steering_angle_));
}

void VescDriver::timerCallback(const ros::SteadyTimerEvent& event) {
  static const double kMaxInitPeriod = 2.0;
  static const double kTStart = ros::WallTime::now().toSec();

  if (kDebug) printf("TIMER CALLBACK\n");
  checkCommandTimeout();
  // VESC interface should not unexpectedly disconnect, but test for it anyway
  CHECK(vesc_.isConnected()) 
      << "Unexpectedly disconnected from serial port.";

  /*
   * Driver state machine, modes:
   *  INITIALIZING - request and wait for vesc version
   *  OPERATING - receiving commands from subscriber topics
   */
  if (driver_mode_ == MODE_INITIALIZING) {
    CHECK_LE(ros::WallTime::now().toSec() - kTStart, kMaxInitPeriod) 
        << "FAIL: Timed out while trying to initialize VESC.\n";

    if (kDebug) printf("INITIALIZING\n");
    // request version number, return packet will update the internal version
    // numbers
    vesc_.requestFWVersion();
    if (fw_version_major_ >= 0 && fw_version_minor_ >= 0) {
      printf("Connected to VESC with firmware version %d.%d\n",
             fw_version_major_, fw_version_minor_);
      driver_mode_ = MODE_OPERATING;
    }
  } else if (driver_mode_ == MODE_OPERATING) {
    
    sendDriveCommands();
    if (kDebug) printf("OPERATING\n");
    // poll for vesc state (telemetry)
    vesc_.requestState();
  } else {
    if (kDebug) printf("FAIL: UNKNOWN STATE!\n");
    // unknown mode, how did that happen?
    assert("unknown driver mode");
  }
}

void VescDriver::updateOdometry(float rpm, float steering_angle) {
  // TODO: calculate speed based on tachometer as opposed to rpm

  // Calcuate linear velocity
  float lin_vel = (rpm - speed_to_erpm_offset_) / speed_to_erpm_gain_;
  // Clamp velocity to zero for minuscule values - a VESC drift issue.
  if (fabs(lin_vel) < 0.01) {
    lin_vel = 0.0;
  }
  // Calculate angular velocity
  float turn_radius = 0;
  float rot_vel = 0;
  if (steering_angle != 0) {
    turn_radius = wheelbase_ / tan(steering_angle);
    rot_vel = lin_vel / turn_radius;
  }

  static float position_x = 0;
  static float position_y = 0;
  static float orientation = 0; // theta
  static double last_frame_time = ros::Time::now().toSec();
  ros::Time current_frame_time_ros = ros::Time::now();
  double current_frame_time = current_frame_time_ros.toSec();

  // Update the estimated pose
  double del_t = current_frame_time - last_frame_time;

  // Enforce monotonically increasing time stamps
  if (del_t >= 0) {
    float del_x = lin_vel * del_t * cos(orientation);
    float del_y = lin_vel * del_t * sin(orientation);
    float del_theta = rot_vel * del_t;

    position_x = position_x + del_x;
    position_y = position_y + del_y;
    orientation = math_util::AngleMod(orientation + del_theta);

    // Create an odometry message
    odom_msg_.twist.twist.linear.x = lin_vel;
    odom_msg_.twist.twist.angular.z = rot_vel;
    odom_msg_.pose.pose.position.x = position_x;
    odom_msg_.pose.pose.position.y = position_y;
    odom_msg_.pose.pose.orientation.w = cos(0.5 * orientation);
    odom_msg_.pose.pose.orientation.z = sin(0.5 * orientation);
    odom_pub_.publish(odom_msg_);
  } else {
    printf("Odometry messages received out of order.\n") ;
  }
  last_frame_time = current_frame_time;
}

void VescDriver::vescPacketCallback(const boost::shared_ptr<VescPacket const>&
packet)
{
  if (packet->name() == "Values") {
    boost::shared_ptr<VescPacketValues const> values =
      boost::dynamic_pointer_cast<VescPacketValues const>(packet);

    state_msg_.header.stamp = ros::Time::now();
    state_msg_.state.voltage_input = values->v_in();
    state_msg_.state.temperature_pcb = values->temp_pcb();
    state_msg_.state.current_motor = values->current_motor();
    state_msg_.state.current_input = values->current_in();
    state_msg_.state.speed = values->rpm();
    state_msg_.state.duty_cycle = values->duty_now();
    state_msg_.state.charge_drawn = values->amp_hours();
    state_msg_.state.charge_regen = values->amp_hours_charged();
    state_msg_.state.energy_drawn = values->watt_hours();
    state_msg_.state.energy_regen = values->watt_hours_charged();
    state_msg_.state.displacement = values->tachometer();
    state_msg_.state.distance_traveled = values->tachometer_abs();
    state_msg_.state.fault_code = values->fault_code();
    state_pub_.publish(state_msg_);

    car_status_msg_.header.stamp = ros::Time::now();
    car_status_msg_.battery_voltage = values->v_in();
    car_status_msg_.status = static_cast<uint8_t>(drive_mode_);
    car_status_pub_.publish(car_status_msg_);

    updateOdometry(values->rpm(), last_steering_angle_);

  }
  else if (packet->name() == "FWVersion") {
    boost::shared_ptr<VescPacketFWVersion const> fw_version =
      boost::dynamic_pointer_cast<VescPacketFWVersion const>(packet);
    // todo: might need lock here
    fw_version_major_ = fw_version->fwMajor();
    fw_version_minor_ = fw_version->fwMinor();
  } else {
    fprintf(stderr, "Unknown packet type: %s\n", packet->name().c_str());
  }
}

float VescDriver::CalculateSteeringAngle(float lin_vel, float rot_vel) {
  float steering_angle = 0.0;
  if (rot_vel == 0) {
    return steering_angle;
  }

  float turn_radius = lin_vel / rot_vel;
  steering_angle = std::atan(wheelbase_ / turn_radius);
  return steering_angle;
}

void VescDriver::ackermannCurvatureCallback(
    const amrl_msgs::AckermannCurvatureDriveMsg& cmd) {
  t_last_command_ = ros::WallTime::now().toSec();
  if (isAutonomous()) {
    mux_drive_speed_ = cmd.velocity;
    const float rot_vel = cmd.velocity * cmd.curvature;
    mux_steering_angle_ = CalculateSteeringAngle(mux_drive_speed_, rot_vel);
  }
}

bool VescDriver::isAutonomous(){
  return drive_mode_ == kAutonomousDrive || drive_mode_ == kAutonomousContinuousDrive;
}
int VescDriver::toggleState(int curToggleState){
  int prevToggleState = prevToggleState_;
  prevToggleState_ = curToggleState;
  if(prevToggleState == 0 && curToggleState == 1){
    return kToggleOn;
  }
  else if(prevToggleState == 1 && curToggleState == 0){
    return kToggleOff;
  }
  else{
    return kNoToggle;
  }
}
} // namespace vesc_driver
