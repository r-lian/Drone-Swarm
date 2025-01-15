#ifndef DRONE_H

#define DRONE_H

#include <tuple>
#include <cmath>
#include <memory>
#include <future>
#include <thread>
#include <chrono>
#include <iostream>

#include <stdlib.h>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <command_line_argument_parser/command_line_argument_parser.h>
#include <mutex/mutex.h>

using namespace std;
using namespace mavsdk;

class Drone {
protected:
	Mutex lock;
	// Custom drone properties.
	string connection_url = "";
	bool _is_connected_to_flight_controller = false;
	// PX4 system properties.
	ConnectionResult connection_result;
	double callback_frequency_in_hz = 1.0;
	float heading = 0.0f;
	// double earths_radius = 6378137.0l; // at equator
	// double earths_radius = 6356752.3l; // at pole
	// double earths_radius = 6371008.8l; // mean radius
	// double earths_radius = 6371007.2l; // authalic (perfect sphere)
	// double earths_radius = 6371000.8l; // volumetric
	long double earths_radius = 6370994.0l; // global mean
	// double earths_radius = 5940990.0l; // tuned simulator value
public:
	shared_ptr<Mavsdk> _mavsdk;
	shared_ptr<System> system;
	shared_ptr<Offboard> offboard;
	shared_ptr<Action> action;
	shared_ptr<Telemetry> telemetry;

	Telemetry::Position position;
	Telemetry::Position home;
	bool in_air = false;
	Telemetry::LandedState landed_state;
 	bool is_armed = false;

	Telemetry::Quaternion attitude_quaternion;
	Telemetry::EulerAngle attitude_euler_angle;
	Telemetry::AngularVelocityBody attitude_angular_velocity_body;
	Telemetry::Quaternion camera_attitude_quaternion;
	Telemetry::EulerAngle camera_attitude_euler_angle;
	Telemetry::VelocityNed velocity_ned;
	Telemetry::GpsInfo gps_info;
	Telemetry::Battery battery;
	Telemetry::FlightMode flight_mode = Telemetry::FlightMode::Unknown;
	Telemetry::Health health;
	Telemetry::RcStatus rc_status;
	Telemetry::StatusText status_text;
	Telemetry::ActuatorControlTarget actuator_control_target;
	Telemetry::ActuatorOutputStatus actuator_output_status;
	Telemetry::Odometry odometry;
	Telemetry::PositionVelocityNed position_velocity_ned;
	Telemetry::GroundTruth ground_truth;
	Telemetry::FixedwingMetrics fixed_wing_metrics;
	Telemetry::Imu imu;
	bool health_all_ok = false;
	uint64_t unix_epoch_time;
	Telemetry::DistanceSensor distance_sensor;

	string get_connection_url();
	ConnectionResult get_connection_result();

	bool is_connected_to_flight_controller();
	Telemetry::LandedState get_landed_state();

	int get_callback_frequency_in_hz();

	Drone(string connection_url);
	// Drone(string connection_url, chrono::seconds _callback_frequency);
	bool wait_until_healthy(chrono::seconds timeout);

	// Arms and starts offboard mode
	bool arm();

	bool wait_until_in_air(chrono::seconds timeout);
	bool initialize_offboard_mode(int timeout_in_seconds);
	bool wait_for_offboard_mode(int timeout_in_seconds);

	// Must be implemented in drone class because ned reference frame
	// is zeroed at home position.
	std::tuple<double, double, float> ned_to_gpsa(
		float _north_m,
		float _east_m,
		float _down_m
	);
	std::tuple<float, float, float> gpsa_to_ned(
		double _latitude_deg,
		double _longitude_deg,
		float _absolute_altitude_m
	);
	// std::tuple<float, float, float> ned_to_frdy(
	// 	float _forward_m,
	// 	float _right_m,
	// 	float _down_m
	// );

	bool action_goto_gpsay_async(
		double _latitude_deg,
		double _longitude_deg,
		float _absolute_altitude_m,
		float _yaw_deg,
		float _speed_m_s
	);
	bool action_goto_gpsay(
		double _latitude_deg,
		double _longitude_deg,
		float _absolute_altitude_m,
		float _yaw_deg,
		float _speed_m_s
	);
	bool action_goto_nedy_async(
		float north_m,
		float east_m,
		float down_m,
		float yaw_deg,
		float speed
	);
	bool action_goto_nedy(
		float north_m,
		float east_m,
		float down_m,
		float yaw_deg,
		float speed
	);
	bool offboard_goto_nedy_async(
		float _north_m,
		float _east_m,
		float _down_m,
		float _yaw_deg,
		float _speed_m_s
	);
	bool offboard_goto_nedy(
		float _north_m,
		float _east_m,
		float _down_m,
		float _yaw_deg,
		float _speed_m_s
	);
	bool offboard_goto_gpsay_async(
		double _latitude_deg,
		double _longitude_deg,
		float _absolute_altitude_m,
		float _yaw_deg,
		float _speed_m_s
	);
	bool offboard_goto_gpsay(
		double _latitude_deg,
		double _longitude_deg,
		float _absolute_altitude_m,
		float _yaw_deg,
		float _speed_m_s
	);

	// Special method.
	bool offboard_velocity_goto_nedy(float north_m, float east_m, float down_m, float yaw_deg, float speed);

	// Forward Right Down Yaw Reference Frame.
	bool goto_frdy(float _forward_m, float _right_m, float _down_m, float _yaw_deg, float speed);
	void unlock();
	bool takeoff();
	bool land();
};

#endif
