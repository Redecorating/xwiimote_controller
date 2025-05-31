/*
 * XWIIMOTE CONTROLLER CLASS
 *
 * Copyright (c) 2020-2021 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of xwiimote_controller project.
 * 
 * All rights reserved.
 *
 */

#ifndef XWIIMOTE_CONTROLLER_H
#define XWIIMOTE_CONTROLLER_H

// C
#include <stdio.h>
#include <poll.h>
extern "C" {
	#include "xwiimote.h"
}

// C++
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/joy_feedback_array.hpp>

// ROS
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <wiimote_msgs/msg/state.hpp>

// TODO namespace
class XWiimoteController : public rclcpp_lifecycle::LifecycleNode {
public:
	explicit XWiimoteController(const std::string & node_name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State &) override;

	~XWiimoteController();
	int openInterface();
	int runInterface();
	void closeInterface();
private:
	rclcpp_lifecycle::LifecyclePublisher<wiimote_msgs::msg::State>::SharedPtr wiimoteStatePub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Joy>::SharedPtr joyPub_, wiimoteNunchukPub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::BatteryState>::SharedPtr batteryPub_;
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::JoyFeedbackArray>> joySetFeedbackSub_;
	rclcpp::Time rumbleEnd_;

  rclcpp::Logger logger_;

  //std::shared_ptr<ParamListener> param_listener_;

	int deviceIdx_;
	std::string devicePath_;
	struct xwii_iface *iface_;
	bool wiimoteCalibrated_, wiimoteConnected_;
	bool buttons_[15], nunchukButtons_[2], leds_[4], rumbleState_;
	float batteryPercent_, nunchukJoystick_[2], nunchuckAcceleration_[3], acceleration_[3], angularVelocity_[3];
	float accelerationCal_[3];

	void getParams();
	void joySetFeedbackCallback(sensor_msgs::msg::JoyFeedbackArray::ConstSharedPtr feedback);
	void publishBattery();
	void publishJoy();
	void publishWiimoteState();
	void publishWiimoteNunchuk();

	// Xwiimote related
	char *getDevice(int num);
	void initializeWiimoteState();
	void readLed();
	void readBattery();
	void toggleRumble(bool on);
	void setLed(unsigned int ledIdx, bool on);
	void setRumble(double duration);
	bool isPresentNunchuk();
	bool isPresentMotionPlus();
	void checkFactoryCalibrationData();

	// Convert wiimote accelerator readings from g's to m/sec^2:
	const double EARTH_GRAVITY_ = 9.80665;  // m/sec^2 @sea_level
};

#endif  // XWIIMOTE_CONTROLLER_H
