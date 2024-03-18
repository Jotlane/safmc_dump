/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */
#include <memory>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <math.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

using std::placeholders::_1;

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control")
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		// Publishers
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

		// Subscribers
		vehicle_control_mode_subscriber_ = this->create_subscription<VehicleControlMode>("/fmu/out/vehicle_control_mode", qos, std::bind(&OffboardControl::vehicle_control_mode_callback, this, _1));
		keyboard_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&OffboardControl::keyboard_callback, this, _1));
		odometry_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("/fmu/out/vehicle_odometry", 10, std::bind(&OffboardControl::keyboard_callback, this, _1));
		camera_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("/cam0_coordinates", 10, std::bind(&OffboardControl::keyboard_callback, this, _1));
		
		offboard_setpoint_counter_ = 0;
		is_offboard_active = false;
		is_offboard_already_active = false;

		drone.forward = 0.0;
		drone.right = 0.0;
		drone.up = 0.0;
		drone.yaw = 0.0;

		auto timer_callback = [this]() -> void {

			// Arm vehicle if offboard mode has been activated from RC
			if (is_offboard_active && is_offboard_already_active == false) {
				RCLCPP_INFO(this->get_logger(), "Offboard Mode Activated");
				RCLCPP_INFO(this->get_logger(), "Arming Vehicle");
				// Arm the vehicle
				this->arm();
				is_offboard_already_active = true; // this ensures vehicle is only armed once.
			}

			if (!is_offboard_already_active) {
				RCLCPP_INFO(this->get_logger(), "Waiting for Offboard Mode to Activate");
			}

			// offboard_control_mode needs to be paired with trajectory_setpoint
			// publish_offboard_control_mode() has to be called a few times before PX4 lets us arm
			publish_offboard_control_mode();


			// start timer after offboard mode has been activated
			if (offboard_setpoint_counter_ < 2400 && is_offboard_active) {
				
				if (offboard_setpoint_counter_ < 200) {
					RCLCPP_INFO(this->get_logger(), "Taking Off");
					publish_trajectory_setpoint(0.0, 0.0, 1.0, M_PI/2);
					drone.up = 1.0;
				} else if (offboard_setpoint_counter_ > 200) {

					// if height setpoint is below 0
					if (drone.up < 0) {
						RCLCPP_INFO(this->get_logger(), "Landing");
						publish_land_command();
					} else {
						publish_trajectory_setpoint(drone.forward, drone.right, drone.up, drone.yaw);
					}
				}

				offboard_setpoint_counter_++;
			} 
		};
		timer_ = this->create_wall_timer(100ms, timer_callback);
	}

	void arm();
	void disarm();

private:

	struct Setpoint {
		float forward;
		float right;
		float up;
		float yaw;
	};

	Setpoint drone;

	// Check if offboard mode is turned on
	void vehicle_control_mode_callback(const VehicleControlMode & msg) {
		RCLCPP_INFO(this->get_logger(), "%d", msg.flag_control_offboard_enabled);
		is_offboard_active = msg.flag_control_offboard_enabled;
	}

	void keyboard_callback(const geometry_msgs::msg::Twist & msg) {
		drone.forward += msg.linear.x;
		drone.right -= msg.linear.y;
		
		float new_height = drone.up + msg.linear.z;

		// Limit height to 1m
		if (new_height > 1.0) {
			drone.up = 1.0;
		} else {
			drone.up = new_height;
		}

		// degree to radian
		float new_yaw = drone.yaw - msg.angular.z * 10 * 0.0174533;

		if (new_yaw > M_PI) {
			drone.yaw = new_yaw - (2 * M_PI);
		} else if (new_yaw < M_PI) {
			drone.yaw = new_yaw + (2 * M_PI);
		} else {
			drone.yaw = new_yaw;
		}
	}

	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Subscription<VehicleControlMode>::SharedPtr vehicle_control_mode_subscriber_;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr keyboard_subscriber_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent
	bool is_offboard_active;
	bool is_offboard_already_active;

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint(float forward, float right, float up, float yaw);
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void publish_take_off_command(float height);
	void publish_land_command();
};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint(float forward, float right, float up, float yaw)
{
	TrajectorySetpoint msg{};
	msg.position = {-right, forward, -up};
	msg.yaw = yaw; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

/**
 * @brief Land at location
*/
void OffboardControl::publish_land_command()
{
	VehicleCommand msg{};
	msg.command = VehicleCommand::VEHICLE_CMD_NAV_LAND;
	msg.param4 = NAN; // Yaw angle (if magnetometer present), ignored without magnetometer
	msg.param5 = NAN; // Latitude
	msg.param6 = NAN; // Logitude
	msg.param7 = 0.0; // Altitude
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}
