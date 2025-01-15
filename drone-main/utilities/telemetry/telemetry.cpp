// Helper functions
#include "helpers.h"

// System libraries
#include <memory>
#include <thread>
#include <cstdint>
#include <stdlib.h>

void subscriptions(Drone &drone, double sub_freq);

// char seperator[] = "--------------------------------------------------";

int main(int argc, char** argv)
{
	Command_Line_Argument_Parser parser(argc, argv);
	map<string, string> flag_arguments = parser.get_flag_arguments();

	Drone drone(flag_arguments["--connection_url"]);
	if (
		(!drone.is_connected_to_flight_controller())           ||
		(flag_arguments["--time_to_exit_in_ms"].empty())       ||
		(flag_arguments["--sampling_frequency_in_hz"].empty())
	) {
		cout	<< parser.get_command() <<
				" --connection_url " << "<connection_url>" <<
				" --time_to_exit_in_ms " << "<time_to_exit>" <<
				" --sampling_frequency_in_hz " << "<sampling_frequency>" << "\n"
			<< "<connection_url> format should be :\n"
			<< "For TCP : tcp://[server_host][:server_port]\n"
			<< "For UDP : udp://[bind_host][:bind_port]\n"
			<< "For Serial : serial:///path/to/serial/dev[:baudrate]\n"
			<< "For example, to connect to the simulator use URL: udp://:14540\n" ;
		return -1;
	}

	double time_to_exit_in_milliseconds = 5000.0;
	double sampling_frequency_in_hz = 1.0;

	time_to_exit_in_milliseconds = stod(flag_arguments["--time_to_exit_in_milliseconds"], nullptr);
	sampling_frequency_in_hz = stod(flag_arguments["--sampling_frequency_in_hz"], nullptr);

	cout << "time_to_exit_in_milliseconds = " << time_to_exit_in_milliseconds << endl;
	cout << "sampling_frequency_in_hz = " << sampling_frequency_in_hz << endl;
	subscriptions(drone, sampling_frequency_in_hz);

	// Check to see if subscription stuff still occurs.
	this_thread::sleep_for(chrono::milliseconds(int(time_to_exit_in_milliseconds)));

	return 0;
}

void subscriptions(Drone &drone, double rate_hz)
{
	drone.telemetry->set_rate_position(rate_hz);
	drone.telemetry->set_rate_home(rate_hz);
	drone.telemetry->set_rate_in_air(rate_hz);
	drone.telemetry->set_rate_landed_state(rate_hz);
	drone.telemetry->set_rate_velocity_ned(rate_hz);
	drone.telemetry->set_rate_gps_info(rate_hz);
	drone.telemetry->set_rate_battery(rate_hz);
	drone.telemetry->set_rate_rc_status(rate_hz);
	drone.telemetry->set_rate_actuator_control_target(rate_hz);
	drone.telemetry->set_rate_actuator_output_status(rate_hz);
	drone.telemetry->set_rate_odometry(rate_hz);
	drone.telemetry->set_rate_position_velocity_ned(rate_hz);
	drone.telemetry->set_rate_ground_truth(rate_hz);
	drone.telemetry->set_rate_fixedwing_metrics(rate_hz);
	drone.telemetry->set_rate_imu(rate_hz);
	drone.telemetry->set_rate_unix_epoch_time(rate_hz);
	drone.telemetry->set_rate_distance_sensor(rate_hz);

	drone.telemetry->set_rate_attitude(rate_hz);
	drone.telemetry->set_rate_camera_attitude(rate_hz);

	cout << seperator << endl;

	drone.telemetry->subscribe_position( [&](Telemetry::Position _position){
		if ( drone.position != _position ) {
			drone.position = _position;
			cout << "drone.position" << endl;
			cout << drone.position << endl;
			cout << seperator << endl;
		}
	});
//	drone.telemetry->subscribe_home( [&](Telemetry::Position _home){
//		if ( drone.home != _home ) {
//			drone.home = _home;
//			cout << "drone.home" << endl;
//			cout << drone.home << endl;
//			cout << seperator << endl;
//		}
//	} );
//	drone.telemetry->subscribe_in_air( [&](bool _in_air){
//		if ( drone.in_air != _in_air ) {
//			drone.in_air = _in_air;
//			cout << "drone.in_air = " <<  ( drone.in_air ? "true" : "false" ) << endl;
//			cout << seperator << endl;
//		}
//	});
//	drone.telemetry->subscribe_landed_state( [&](Telemetry::LandedState _landed_state){
//		if ( drone.landed_state != _landed_state ) {
//			drone.landed_state = _landed_state;
//			cout << "drone.landed_state" << endl;
//			cout << drone.landed_state << endl;
//			cout << seperator << endl;
//		}
//	});
//	drone.telemetry->subscribe_armed( [&](bool _is_armed) {
//		if ( drone.is_armed != _is_armed ) {
//			drone.is_armed = _is_armed;
//			cout << "drone.is_armed = " << (drone.is_armed ? "true" : "false") << endl;
//			cout << seperator << endl;
//		}
//	});
//	// drone.telemetry->subscribe_attitude_quaternion( [&](Telemetry::Quaternion _quaternion){
//	// 	if ( drone.attitude_quaternion != _quaternion ) {
//	// 		drone.attitude_quaternion = _quaternion;
//	// 		cout << "drone.attitude_quaternion" << endl;
//	// 		cout << drone.attitude_quaternion << endl;
//	// 		cout << seperator << endl;
//	// 	}
//	// });
	drone.telemetry->subscribe_attitude_euler( [&](Telemetry::EulerAngle _euler_angle){
		if ( drone.attitude_euler_angle != _euler_angle ) {
			drone.attitude_euler_angle = _euler_angle;
			cout << "drone.attitude_euler_angle" << endl;
			cout << drone.attitude_euler_angle << endl;
			cout << seperator << endl;
			float front_to_north_yaw_deg = -1 * drone.attitude_euler_angle.yaw_deg;
			cout << "(60m forward, 70m right) = "
				<< "(" << 60 * cos( front_to_north_yaw_deg ) + 70 * sin( front_to_north_yaw_deg ) << ","
				<< -1 * 60 * sin( front_to_north_yaw_deg ) + 70 * cos ( front_to_north_yaw_deg ) << ")\n";
		}
	});
//	// drone.telemetry->subscribe_attitude_angular_velocity_body( [&](Telemetry::AngularVelocityBody _attitude_angular_velocity_body){
//	// 	if ( drone.attitude_angular_velocity_body != _attitude_angular_velocity_body ) {
//	// 		drone.attitude_angular_velocity_body = _attitude_angular_velocity_body;
//	// 		cout << "drone.attitude_angular_velocity_body" << endl;
//	// 		cout << drone.attitude_angular_velocity_body << endl;
//	// 		cout << seperator << endl;
//	// 	}
//	// });
//	// drone.telemetry->subscribe_camera_attitude_quaternion( [&](Telemetry::Quaternion _quaternion){
//	// 	if ( drone.camera_attitude_quaternion != _quaternion ) {
//	// 		drone.camera_attitude_quaternion = _quaternion;
//	// 		cout << "drone.camera_attitude_quaternion" << endl;
//	// 		cout << drone.camera_attitude_quaternion << endl;
//	// 		cout << seperator << endl;
//	// 	}
//	// });
//	// drone.telemetry->subscribe_camera_attitude_euler( [&](Telemetry::EulerAngle _euler_angle) {
//	// 	if ( drone.camera_attitude_euler_angle != _euler_angle ) {
//	// 		drone.camera_attitude_euler_angle = _euler_angle;
//	// 		cout << "drone.camera_attitude_euler_angle" << endl;
//	// 		cout << drone.camera_attitude_euler_angle << endl;
//	// 		cout << seperator << endl;
//	// 	}
//	// });
//	drone.telemetry->subscribe_velocity_ned( [&](Telemetry::VelocityNed _velocity_ned){
//		if ( drone.velocity_ned != _velocity_ned ) {
//			drone.velocity_ned = _velocity_ned;
//			cout << "drone.velocity_ned" << endl;
//			cout << drone.velocity_ned << endl;
//			cout << seperator << endl;
//		}
//	});
//	drone.telemetry->subscribe_gps_info( [&](Telemetry::GpsInfo _gps_info){
//		if ( drone.gps_info != _gps_info ) {
//			drone.gps_info = _gps_info;
//			cout << "drone.gps_info" << endl;
//			cout << drone.gps_info << endl;
//			cout << seperator << endl;
//		}
//	});
//	drone.telemetry->subscribe_battery([&](Telemetry::Battery _battery) {
//		if ( drone.battery != _battery ) {
//			drone.battery = _battery;
//			cout << "drone.battery" << endl ;
//			cout << drone.battery << endl ;
//			cout << seperator << endl;
//		}
//	});
//	drone.telemetry->subscribe_flight_mode([&](Telemetry::FlightMode _flight_mode) {
//		if ( drone.flight_mode != _flight_mode ) {
//			drone.flight_mode = _flight_mode;
//			cout << "drone.flight_mode =  " << drone.flight_mode << endl ;
//			cout << seperator << endl;
//		}
//	} ) ;
//	drone.telemetry->subscribe_health([&](Telemetry::Health _health){
//		if ( drone.health != _health ) {
//			drone.health = _health;
//			cout << "drone.health = " << drone.health << endl;
//			cout << seperator << endl;
//		}
//	});
//	drone.telemetry->subscribe_rc_status([&](Telemetry::RcStatus _rc_status){
//		if ( drone.rc_status != _rc_status ) {
//			drone.rc_status = _rc_status;
//			cout << "drone.rc_status = " << drone.rc_status << endl;
//			cout << seperator << endl;
//		}
//	});
//	drone.telemetry->subscribe_status_text([&](Telemetry::StatusText _status_text){
//		if ( drone.status_text != _status_text ) {
//			drone.status_text = _status_text;
//			cout << "drone.status_text = " << drone.status_text << endl;
//			cout << seperator << endl;
//		}
//	});
//	drone.telemetry->subscribe_actuator_control_target(
//		[&](Telemetry::ActuatorControlTarget _actuator_control_target){
//			if ( drone.actuator_control_target != _actuator_control_target ) {
//				drone.actuator_control_target = _actuator_control_target;
//				cout << drone.actuator_control_target << endl;
//				cout << seperator << endl;
//			}
//		}
//	);
//	drone.telemetry->subscribe_actuator_output_status(
//		[&](Telemetry::ActuatorOutputStatus _actuator_output_status){
//			if ( drone.actuator_output_status != _actuator_output_status ) {
//				drone.actuator_output_status = _actuator_output_status;
//				cout << "drone.actuator_output_status" << endl;
//				cout << drone.actuator_output_status << endl;
//				cout << seperator << endl;
//			}
//		}
//	);
//	drone.telemetry->subscribe_odometry([&](Telemetry::Odometry _odometry){
//		if ( drone.odometry != _odometry ) {
//			drone.odometry = _odometry;
//			cout << "drone.odometry" << endl;
//			cout << drone.odometry << endl;
//			cout << seperator << endl;
//		}
//	});
//	drone.telemetry->subscribe_position_velocity_ned( [&](Telemetry::PositionVelocityNed _position_velocity_ned){
//		if ( drone.position_velocity_ned != _position_velocity_ned ) {
//			drone.position_velocity_ned = _position_velocity_ned;
//			cout << "drone.position_velocity_ned" << endl;
//			cout << drone.position_velocity_ned << endl;
//			cout << seperator << endl;
//		}
//	});
//	drone.telemetry->subscribe_ground_truth([&](Telemetry::GroundTruth _ground_truth){
//		if ( drone.ground_truth != _ground_truth ) {
//			drone.ground_truth = _ground_truth;
//			cout << "drone.ground_truth" << endl;
//			cout << drone.ground_truth << endl;
//			cout << seperator << endl;
//		}
//	});
//	drone.telemetry->subscribe_fixedwing_metrics([&](Telemetry::FixedwingMetrics _fixed_wing_metrics){
//		if ( drone.fixed_wing_metrics != _fixed_wing_metrics ) {
//			drone.fixed_wing_metrics = _fixed_wing_metrics;
//			cout << "drone.fixed_wing_metrics" << endl;
//			cout << drone.fixed_wing_metrics << endl;
//			cout << seperator << endl;
//		}
//	});
//      drone.telemetry->subscribe_imu([&](Telemetry::Imu _imu){
//      	if ( drone.imu != _imu ) {
//      		drone.imu = _imu;
//      		cout << "drone.imu" << endl;
//      		cout << drone.imu << endl;
//      		Telemetry::MagneticFieldFrd magnetometer = drone.imu.magnetic_field_frd;
//
//      		float heading = atan2(magnetometer.right_gauss, magnetometer.forward_gauss) * 180 / M_PI;
//      		cout << seperator << endl;
//      		cout << "heading = " << heading << endl;
//      		cout << seperator << endl;
//      	}
//      });
//	drone.telemetry->subscribe_health_all_ok([&](bool _health_all_ok){
//		if ( drone.health_all_ok != _health_all_ok ) {
//			drone.health_all_ok = _health_all_ok;
//			cout << "drone.health_all_ok = " << (drone.health_all_ok ? "true" : "false") << endl ;
//		}
//	} ) ;
//	drone.telemetry->subscribe_unix_epoch_time([&](uint64_t _unix_epoch_time){
//		if ( drone.unix_epoch_time != _unix_epoch_time ) {
//			drone.unix_epoch_time = _unix_epoch_time;
//			cout << "drone.unix_epoch_time = " << drone.unix_epoch_time << endl ;
//		}
//	});
//	drone.telemetry->subscribe_distance_sensor([&](Telemetry::DistanceSensor _distance_sensor){
//		if ( drone.distance_sensor != _distance_sensor ) {
//			drone.distance_sensor = _distance_sensor;
//			cout << "drone.distance_sensor = " << drone.distance_sensor << endl;
//		}
//	});

}
