#include "drone.h"

Drone::Drone(string url) {
	cout << "Initializing all drone attributes." << endl;
	health_all_ok = false;
	flight_mode = Telemetry::FlightMode::Unknown ;
	is_armed = false ;
	in_air = false;
	_is_connected_to_flight_controller = false ;
	_mavsdk = shared_ptr<Mavsdk>(new Mavsdk());

	cout << "Attempting to connect." << endl;
	if ( url.empty() ) {
		cout << "no connection url\n";
		return;
	}
	cout << "url = " << url << endl;
	connection_result = _mavsdk->add_any_connection(url);

	cout << "Before connection_result comparison." << endl;

	if (connection_result != ConnectionResult::Success) {
		cout << "connection_result = " << connection_result << endl
			<< "Could not connect to flight contoller. " << endl;
		return;
	}

	connection_url = url;

	cout << "Waiting to discover system..." << endl;

	std::promise<void> prom;
	std::future<void> fut = prom.get_future();

	_mavsdk->subscribe_on_new_system([&]() {
		system = _mavsdk->systems().at(0);
		if (!system->is_connected()) return;

		cout << "setting promise\n";
		prom.set_value();
		// prom.set_value();
		_mavsdk->subscribe_on_new_system(nullptr);
	});

	if ( !(fut.wait_for(chrono::seconds(5)) == future_status::ready) ) {
		cout << "Could not connect to flight controller." << endl;
		_is_connected_to_flight_controller = false;
		_mavsdk->subscribe_on_new_system(nullptr);
		return;
	}

	_is_connected_to_flight_controller = true;
	cout << "Connected to flight controller." << endl;
	_mavsdk->subscribe_on_new_system(nullptr);

	telemetry = shared_ptr<Telemetry>(new Telemetry(system));
	action = shared_ptr<Action>(new Action(system));
	offboard = shared_ptr<Offboard>(new Offboard(system));

	// wait_until_healthy(chrono::seconds(5));
	// cout << "Created telemetry, action, and offboard shared pointers.\n";
	// Somehow calling telemetry set rate is causing double promise calls?

	// telemetry->set_rate_position(callback_frequency_in_hz);
	// telemetry->set_rate_home(callback_frequency_in_hz);
	// telemetry->set_rate_in_air(callback_frequency_in_hz);
	// telemetry->set_rate_landed_state(callback_frequency_in_hz);
	// telemetry->set_rate_velocity_ned(callback_frequency_in_hz);
	// telemetry->set_rate_gps_info(callback_frequency_in_hz);
	// telemetry->set_rate_battery(callback_frequency_in_hz);
	// telemetry->set_rate_rc_status(callback_frequency_in_hz);
	// telemetry->set_rate_actuator_control_target(callback_frequency_in_hz);
	// telemetry->set_rate_actuator_output_status(callback_frequency_in_hz);
	// telemetry->set_rate_odometry(callback_frequency_in_hz);
	// telemetry->set_rate_position_velocity_ned(callback_frequency_in_hz);
	// telemetry->set_rate_ground_truth(callback_frequency_in_hz);
	// telemetry->set_rate_fixedwing_metrics(callback_frequency_in_hz);
	// telemetry->set_rate_imu(callback_frequency_in_hz);
	// telemetry->set_rate_unix_epoch_time(callback_frequency_in_hz);
	// telemetry->set_rate_distance_sensor(callback_frequency_in_hz);

	// telemetry->set_rate_attitude(callback_frequency_in_hz);
	// telemetry->set_rate_camera_attitude(callback_frequency_in_hz);
}

string Drone::get_connection_url() { return connection_url; }
ConnectionResult Drone::get_connection_result() { return connection_result; }

bool Drone::is_connected_to_flight_controller() { return _is_connected_to_flight_controller; }

bool Drone::wait_until_in_air(chrono::seconds timeout) {
	promise<void> prom;
	future<void> fut = prom.get_future();
	lock.lock();

	telemetry->subscribe_in_air([&](bool is_in_air) {
		system = _mavsdk->systems().at(0);
		if (!system->is_connected()) return;

		prom.set_value();
		telemetry->subscribe_in_air(nullptr);
	});
	if ( fut.wait_for(timeout) == future_status::timeout )
		return false;

	lock.unlock();
	return true;
}

// returns false if healthy state is not possible.
// returns true when healthy state 
bool Drone::wait_until_healthy(chrono::seconds timeout) {
	cout << "Waiting for drone to be healthy." << endl;
	health_all_ok = telemetry->health_all_ok();
	health = telemetry->health();

	promise<void> prom;
	future<void> fut = prom.get_future();
	telemetry->subscribe_health_all_ok([&](bool _health_all_ok){
		if ( health_all_ok == true ) {
			prom.set_value();
			telemetry->subscribe_health_all_ok(nullptr);
		}
	} ) ;

	if ( fut.wait_for(timeout) == future_status::ready ) {
		attitude_euler_angle = telemetry->attitude_euler();
		heading = attitude_euler_angle.yaw_deg;
		cout << "Facing " << heading << "deg East of North\n";
		return true;
	}

	if (!health.is_gyrometer_calibration_ok) {
		cout << "Gyro requires calibration." << '\n';
	}
	if (!health.is_accelerometer_calibration_ok) {
		cout << "Accelerometer requires calibration." << '\n';
	}
	if (!health.is_magnetometer_calibration_ok) {
		cout << "Magnetometer (compass) requires calibration." << '\n';
	}
	// if (!health.is_level_calibration_ok) {
	// 	cout << "Level calibration required." << '\n';
	// }
	if (!health.is_global_position_ok) {
		cout << "Global position not ok." << '\n';
	}
	if (!health.is_home_position_ok) {
		cout << "Home position not ok." << '\n';
	}
	if (!health.is_local_position_ok) {
		cout << "Local position not ok." << '\n';
	}

	return false;
}
bool Drone::arm() {
	cout << "Attempting to arm drone.\n";
	Action::Result result = action->arm();
	if ( result == Action::Result::Success ) {
		is_armed = true;
		return true;
	}

	cout << "Arming failed: arming result: " << result << endl;
	return false;
}

int Drone::get_callback_frequency_in_hz(){ return callback_frequency_in_hz; }

bool Drone::initialize_offboard_mode(int timeout_in_seconds) {
	cout << "Initializing offboard mode\n";
	Offboard::VelocityBodyYawspeed stay{};
	offboard->set_velocity_body(stay);

	cout << "timeout_in_seconds = " << timeout_in_seconds << "\n";
	for (
		int timeout_index = 0;
		timeout_index < timeout_in_seconds;
		timeout_index++
	) {
		if ( offboard->start() == Offboard::Result::Success ) {
			cout << "Offboard mode active\n";
			return true;
		}
		cout << "Offboard mode inactive\n";
		std::this_thread::sleep_for(chrono::seconds(1));
	}
	cout << "Out of for loop\n";
	// flight_mode = Telemetry::FlightMode::Offboard;
	return false;
}

bool Drone::wait_for_offboard_mode(int timeout_in_seconds) {
	promise<void> prom;
	future<void> fut = prom.get_future();

	telemetry->subscribe_flight_mode([&](Telemetry::FlightMode new_flight_mode) {
		if ( new_flight_mode == Telemetry::FlightMode::Offboard ) {
			flight_mode = new_flight_mode;
			prom.set_value();
			telemetry->subscribe_flight_mode(nullptr);
		}
		Offboard::VelocityBodyYawspeed stay{};
		offboard->set_velocity_body(stay);
	} ) ;

	if ( fut.wait_for(chrono::seconds(timeout_in_seconds)) == future_status::timeout )
		return false;

//	telemetry->subscribe_flight_mode([&](Telemetry::FlightMode new_flight_mode) {
//		if ( new_flight_mode != Telemetry::FlightMode::Offboard ) {
//			printf("Exiting offboard mode!\n");
//			flight_mode = new_flight_mode;
//			lock.set_value();
//			telemetry->subscribe_flight_mode(nullptr);
//		}
//		Offboard::VelocityBodyYawspeed stay{};
//		offboard->set_velocity_body(stay);
//		exit(EXIT_SUCCESS);
//	} ) ;

	return true;
}

float calculate_magnitude(std::tuple<float, float, float> vector) {
	// cout << "Calculating vector magnitude:\n";
	float magnitude = sqrt(
		pow(std::get<0>(vector), 2.0f) +
		pow(std::get<1>(vector), 2.0f) +
		pow(std::get<2>(vector), 2.0f)
	);
	// cout << "North: " << std::get<0>(vector) << "\n";
	// cout << "East: " << std::get<1>(vector) << "\n";
	// cout << "Down: " << std::get<2>(vector) << "\n";
	// cout << "magnitude: " << magnitude << "\n";
	return magnitude;
}



std::tuple<float, float, float> calculate_unit_vector(std::tuple<float, float, float> vector) {
	// cout << "Calculating Unit Vector:\n";
	float magnitude = calculate_magnitude(vector);
	std::tuple<float, float, float> unit_vector = vector;

	std::get<0>(unit_vector) = std::get<0>(unit_vector) / magnitude;
	std::get<1>(unit_vector) = std::get<1>(unit_vector) / magnitude;
	std::get<2>(unit_vector) = std::get<2>(unit_vector) / magnitude;

	// cout << "North: " << std::get<0>(unit_vector) << "\n";
	// cout << "East: " << std::get<1>(unit_vector) << "\n";
	// cout << "Down: " << std::get<2>(unit_vector) << "\n";

	return unit_vector;
}

std::tuple<float, float, float> operator-(
	std::tuple<float, float, float> lh,
	std::tuple<float, float, float> rh
) {
	std::tuple<float, float, float> result {
		std::get<0>(lh) - std::get<0>(rh),
		std::get<1>(lh) - std::get<1>(rh),
		std::get<2>(lh) - std::get<2>(rh)
	} ;
	return result;
}

std::tuple<float, float, float> operator*(
	float lh,
	std::tuple<float, float, float> rh
) {
	std::tuple<float, float, float> result {
		lh * std::get<0>(rh),
		lh * std::get<1>(rh),
		lh * std::get<2>(rh)
	} ;
	return result;
}

std::tuple<float, float, float> new_velocity(
	std::tuple<float, float, float> from,
	std::tuple<float, float, float> to,
	float speed
) {
	std::tuple<float, float, float> diff = to - from;
	std::tuple<float, float, float> unit_diff = calculate_unit_vector(diff);
	std::tuple<float, float, float> new_v = speed * unit_diff;

	return new_v;
}


std::tuple<double, double, float> Drone::ned_to_gpsa(
	float _north_m,
	float _east_m,
	float _down_m
) {
	// cout << "Attempting to calculate lat/long from meters north/east.\n";

	// Get home and current positions.
	Telemetry::Position _home = telemetry->home();
	while (isnan(_home.latitude_deg)) _home = telemetry->home();
	// Telemetry::Position _position = telemetry->position();

	// cout << "home lat: " << _home.latitude_deg << "\n";
	// cout << "home long: " << _home.longitude_deg << "\n";
	// cout << "home alt: " << _home.absolute_altitude_m << "\n";

	// Convert nedy to rotations
	std::tuple<double, double, float> gps_position;

	long double deg_to_rad = (long double)M_PI / 180.0l;
	long double rad_to_deg = 180.0l / (long double)M_PI;

	long double latitude = (long double)_home.latitude_deg + (long double)_north_m * rad_to_deg / earths_radius;
	// long double latitude = (long double)_home.latitude_deg + (long double)_north_m * deg_to_rad / earths_radius;
	// std::get<0>(gps_position) = _home.latitude_deg +  (double)_north_m * deg_to_rad / (double)earths_radius;
	// std::get<1>(gps_position) = _home.longitude_deg + _east_m * 180.0 / M_PI / earths_longitude_radius;
	// std::get<1>(gps_position) = _home.longitude_deg + (double)_east_m * 180.0 / M_PI / (double)earths_radius / cos( std::get<0>(gps_position) * M_PI / 180.0l );
	// std::get<1>(gps_position) = _home.longitude_deg + ( (long double)_east_m * 180.0l ) / ( (long double)M_PI * earths_radius * cosl( std::get<0>(gps_position) * M_PI / 180.0l ) );

	long double longitude = (long double)_home.longitude_deg + (long double)_east_m * rad_to_deg / ( earths_radius * cosl( latitude * deg_to_rad ) );

	std::get<0>(gps_position) = (double)latitude;
	std::get<1>(gps_position) = (double)longitude;
	std::get<2>(gps_position) = _home.absolute_altitude_m - _down_m;

	// cout << "relative north: " << std::get<0>(gps_position) << "\n";
	// cout << "relative east: " << std::get<1>(gps_position) << "\n";
	// cout << "relative down: " << std::get<2>(gps_position) << "\n";

	return gps_position;
}

std::tuple<float, float, float> Drone::gpsa_to_ned(
	double _latitude_deg,
	double _longitude_deg,
	float _absolute_altitude_m
) {
	// float radius = earths_radius + _absolute_altitude_m;

	long double deg_to_rad = (long double)M_PI / 180.0l;
	long double rad_to_deg = 180.0l / (long double)M_PI;

	// cout << "Attempting to calculate north/east from gps.\n";
	long double latitude_rad = (long double)_latitude_deg * deg_to_rad;
	long double longitude_rad = (long double)_longitude_deg * deg_to_rad;

	Telemetry::Position _home = telemetry->home();
	while (isnan(_home.latitude_deg)) _home = telemetry->home();

	cout << "home lat: " << _home.latitude_deg << "\n";
	cout << "home long: " << _home.longitude_deg << "\n";
	cout << "home alt: " << _home.absolute_altitude_m << "\n";

	cout << "cur lat: " << _latitude_deg << "\n";
	cout << "cur long: " << _longitude_deg << "\n";

	long double lat_change = ((long double)(_latitude_deg - _home.latitude_deg)) * deg_to_rad;
	long double long_change = ((long double)(_longitude_deg - _home.longitude_deg)) * deg_to_rad;

	long double north = lat_change * earths_radius ;
	// long double east = long_change * earths_radius ;
	long double east = long_change * earths_radius * cosl(latitude_rad) ;
	float down = _home.absolute_altitude_m - _absolute_altitude_m;

	std::tuple<float, float, float> ned{
		(float)north,
		(float)east,
		down
	};

	cout << "north: " << north << "\n";
	cout << "east: " << east << "\n";
	cout << "down: " << down << "\n";

	return ned;
}
bool Drone::action_goto_gpsay_async(
	double _latitude_deg,
	double _longitude_deg,
	float _absolute_altitude_m,
	float _yaw_deg,
	float _speed_m_s
) {
	Action::Result result = action->set_maximum_speed(_speed_m_s);
	if ( result != Action::Result::Success ) return false;

	std::pair<Action::Result, float> max_speed = action->get_maximum_speed();
	if ( std::get<0>(max_speed) != Action::Result::Success ) return false;

	// cout << "--------------------------------------------------" << "\n";
	// cout << "max speed: " << std::get<1>(max_speed) << "\n";
	// cout << "latitude: " << _latitude_deg << "\n";
	// cout << "longitude: " << _longitude_deg << "\n";
	// cout << "altitude: " << _absolute_altitude_m << "\n";
	Action::Result action_result = action->goto_location(
		_latitude_deg,
		_longitude_deg,
		_absolute_altitude_m,
		_yaw_deg
	);

	if ( action_result != Action::Result::Success ) return false;

	return true;
}

bool Drone::action_goto_gpsay(
	double _latitude_deg,
	double _longitude_deg,
	float _absolute_altitude_m,
	float _yaw_deg,
	float _speed_m_s
) {
	promise<void> prom;
	future<void> fut = prom.get_future();
	// cout << "Attempting to lock!\n";
	lock.lock();

	Action::Result result = action->set_maximum_speed(_speed_m_s);
	if ( result != Action::Result::Success ) return false;

	std::pair<Action::Result, float> max_speed = action->get_maximum_speed();
	if ( std::get<0>(max_speed) != Action::Result::Success ) return false;

	// cout << "--------------------------------------------------" << "\n";
	// cout << "max speed: " << std::get<1>(max_speed) << "\n";
	// cout << "latitude: " << _latitude_deg << "\n";
	// cout << "longitude: " << _longitude_deg << "\n";
	// cout << "altitude: " << _absolute_altitude_m << "\n";
	action->goto_location(
		_latitude_deg,
		_longitude_deg,
		_absolute_altitude_m,
		_yaw_deg
	);

	int counter = 0;
	const float callback_frequency = 100.0f;
	const float minimum_distance = 1.0f;
	float distance_to_destination = 1000.0f;

	telemetry->set_rate_position(callback_frequency); // 1000 times a sec
	telemetry->subscribe_position( [&](Telemetry::Position _position){

		// cout << "Current Position: " << _position << "\n";

		// cout << "Going to: \n";
		// cout << "lat: " << _latitude_deg << "\n";
		// cout << "long: " << _longitude_deg << "\n";
		// cout << "abs. alt.: " << _absolute_altitude_m << "\n";
		// cout << "yaw_deg: " << _yaw_deg << "\n";

		std::tuple<float, float, float> ned_difference = gpsa_to_ned(
			_latitude_deg,
			_longitude_deg,
			_absolute_altitude_m
		) - gpsa_to_ned(
			_position.latitude_deg,
			_position.longitude_deg,
			_position.absolute_altitude_m
		);

		distance_to_destination = calculate_magnitude(ned_difference);

		counter++;
		if (counter == 100) {
			counter = 0;
			cout << "north difference: " << std::get<0>(ned_difference) << "\n";
			cout << "east difference: " << std::get<1>(ned_difference) << "\n";
			cout << "Distance to destination: " << distance_to_destination << "\n";
		}

		if ( distance_to_destination < minimum_distance || (!lock.is_locked()) ) {
			cout << "distance: " << distance_to_destination << "\n";
			cout << "north difference: " << std::get<0>(ned_difference) << "\n";
			cout << "east difference: " << std::get<1>(ned_difference) << "\n";
			cout << "Distance to destination: " << distance_to_destination << "\n";
			cout << "Reached gps coordinate!\n";
			telemetry->subscribe_position(nullptr);
			lock.unlock();
			prom.set_value();
		}
		return;
	});
	cout << "Waiting to finish goto\n";
	fut.wait();
	
	if ( distance_to_destination < minimum_distance )
		return true;
	return false;
}

bool Drone::action_goto_nedy_async(float _north_m, float _east_m, float _down_m, float _yaw_deg, float _speed) {
	cout << "Attempting action based goto\n";
	std::tuple<double, double, float> gps = ned_to_gpsa(_north_m, _east_m, _down_m);

	Action::Result result = action->set_maximum_speed(_speed);
	if ( result != Action::Result::Success ) return false;

	std::pair<Action::Result, float> max_speed = action->get_maximum_speed();
	if ( std::get<0>(max_speed) != Action::Result::Success ) return false;

	Action::Result action_result = action->goto_location(
		std::get<0>(gps),
		std::get<1>(gps),
		std::get<2>(gps),
		_yaw_deg
	);

	if ( action_result != Action::Result::Success) return false;
	return true;
}

bool Drone::action_goto_nedy(float _north_m, float _east_m, float _down_m, float _yaw_deg, float _speed) {
	promise<void> prom;
	future<void> fut = prom.get_future();
	// cout << "Trying to lock\n";
	lock.lock();
	cout << "Attempting action based goto\n";
	std::tuple<double, double, float> gps = ned_to_gpsa(_north_m, _east_m, _down_m);
	// set speed
	// goto
	// cout << "north_m: " << _north_m << "\n";
	// cout << "east_m: " << _east_m << "\n";
	// cout << "down_m: " << _down_m << "\n";

	Action::Result result = action->set_maximum_speed(_speed);
	if ( result != Action::Result::Success ) return false;

	std::pair<Action::Result, float> max_speed = action->get_maximum_speed();
	if ( std::get<0>(max_speed) != Action::Result::Success ) return false;

	// cout << "max speed: " << std::get<1>(max_speed) << "\n";
	// cout << "latitude: " << std::get<0>(gps) << "\n";
	// cout << "longitude: " << std::get<1>(gps) << "\n";
	// cout << "altitude: " << std::get<2>(gps) << "\n";
	action->goto_location(
		std::get<0>(gps),
		std::get<1>(gps),
		std::get<2>(gps),
		_yaw_deg
	);

	std::tuple<float, float, float> final_position_tuple {_north_m, _east_m, _down_m };
	const float callback_frequency = 100.0f;
	const float minimum_distance = 0.5f;
	float distance_to_destination = 1000.0f;

	telemetry->set_rate_position_velocity_ned(callback_frequency); // 1000 times a sec
	telemetry->subscribe_position_velocity_ned( [&](Telemetry::PositionVelocityNed _position_velocity_ned){
		Telemetry::PositionNed _position = _position_velocity_ned.position;
		Telemetry::VelocityNed _velocity = _position_velocity_ned.velocity;

		// cout << "heading to : \n";
		cout << "north_m: " << _north_m << "\n";
		cout << "east_m: " << _east_m << "\n";
		cout << "down_m: " << _down_m << "\n";

		std::tuple new_position_tuple {
			_position.north_m,
			_position.east_m,
			_position.down_m
		};

		std::tuple velocity_tuple {
			_velocity.north_m_s,
			_velocity.east_m_s,
			_velocity.down_m_s
		};

		float speed = calculate_magnitude(velocity_tuple);

		std::tuple diff = final_position_tuple - new_position_tuple;
		distance_to_destination = calculate_magnitude(diff);

		cout << "distance = " << distance_to_destination << "m\n";
		cout << "current speed: " << speed << "m/s\n";
		// cout << "max speed: " << std::get<1>(max_speed) << "m/s\n";

		if ( distance_to_destination < minimum_distance || (!lock.is_locked())) {
			cout << "Reached nedy coordinate!\n";
			telemetry->subscribe_position_velocity_ned(nullptr);
			lock.unlock();
			prom.set_value();
		}
		return;
	});
	cout << "Waiting to finish goto\n";
	fut.wait();

	if ( distance_to_destination < minimum_distance )
		return true;
	return false;
}

bool Drone::offboard_goto_nedy_async(
	float _north_m,
	float _east_m,
	float _down_m,
	float _yaw_deg,
	float _speed_m_s
) {
	Action::Result result = action->set_maximum_speed(_speed_m_s);
	if ( result != Action::Result::Success ) return false;

	Offboard::PositionNedYaw final_position_ned_yaw{_north_m, _east_m, _down_m, _yaw_deg};
	Offboard::Result offboard_result = offboard->set_position_ned(final_position_ned_yaw);
	if ( offboard_result != Offboard::Result::Success ) return false;
	return true;
}

bool Drone::offboard_goto_nedy(
	float _north_m,
	float _east_m,
	float _down_m,
	float _yaw_deg,
	float _speed_m_s
) {
	promise<void> prom;
	future<void> fut = prom.get_future();
	lock.lock();

	const float callback_frequency = 1000.0f;
	cout << "Attempting to goto nedy coordinate\n";
	telemetry->set_rate_position_velocity_ned(callback_frequency); // 1000 times a sec
	Action::Result result = action->set_maximum_speed(_speed_m_s);
	if ( result != Action::Result::Success ) return false;

	const float minimum_distance = 0.5f;

	Telemetry::PositionVelocityNed pv_ned = telemetry->position_velocity_ned();

	Offboard::PositionNedYaw final_position_ned_yaw{_north_m, _east_m, _down_m, _yaw_deg};
	std::tuple<float, float, float> final_position_tuple {_north_m, _east_m, _down_m};
	std::tuple<float, float, float> start_position_tuple {
		pv_ned.position.north_m,
		pv_ned.position.east_m,
		pv_ned.position.down_m
	};
	std::tuple<float, float, float> overall_position_difference_tuple = final_position_tuple - start_position_tuple;
	float start_distance = calculate_magnitude(overall_position_difference_tuple);
	float distance = 1000.0f;

	// Gives position and velocity in ned frame.
	telemetry->subscribe_position_velocity_ned( [&](Telemetry::PositionVelocityNed _position_velocity_ned){
		offboard->set_position_ned(final_position_ned_yaw);
		Telemetry::PositionNed _position = _position_velocity_ned.position;
		Telemetry::VelocityNed _velocity = _position_velocity_ned.velocity;

		std::tuple<float, float, float> position_tuple {
			_position.north_m,
			_position.east_m,
			_position.down_m
		};

		std::tuple<float, float, float> velocity_tuple {
			_velocity.north_m_s,
			_velocity.east_m_s,
			_velocity.down_m_s
		};

		float velocity_tuple_magnitude = calculate_magnitude(velocity_tuple);

		std::tuple diff = final_position_tuple - position_tuple;
		distance = calculate_magnitude(diff);
		float distance_percent = 100.0f * distance / start_distance;

		cout << "start distance = " << start_distance << "m\n";
		cout << "distance = " << distance << "m\n";
		cout << "percent distance = " << distance_percent << "%\n";
		cout << "current speed: " << velocity_tuple_magnitude << "m/s\n";
		cout << "maximum speed: " << _speed_m_s << "m/s\n";

		if ( distance < minimum_distance || (!lock.is_locked()) ) {
			cout << "Reached nedy coordinate!\n";
			telemetry->subscribe_position_velocity_ned(nullptr);
			lock.unlock();
			prom.set_value();
		}
		return;
	});

	cout << "Waiting to finish goto\n";
	fut.wait();

	if ( distance < minimum_distance )
		return true;

	return false;
}

bool Drone::offboard_goto_gpsay_async(
	double _latitude_deg,
	double _longitude_deg,
	float _absolute_altitude_m,
	float _yaw_deg,
	float _speed_m_s
) {
	std::tuple<float, float, float> ned = gpsa_to_ned(
		_latitude_deg,
		_longitude_deg,
		_absolute_altitude_m
	);
	return offboard_goto_nedy_async(
		std::get<0>(ned),
		std::get<1>(ned),
		std::get<2>(ned),
		_yaw_deg,
		_speed_m_s
	);
}
bool Drone::offboard_goto_gpsay(
	double _latitude_deg,
	double _longitude_deg,
	float _absolute_altitude_m,
	float _yaw_deg,
	float _speed_m_s
) {
	// Convert to nedy then do offboard_goto_nedy?
	std::tuple<float, float, float> ned = gpsa_to_ned(
		_latitude_deg,
		_longitude_deg,
		_absolute_altitude_m
	);
	return offboard_goto_nedy(
		std::get<0>(ned),
		std::get<1>(ned),
		std::get<2>(ned),
		_yaw_deg,
		_speed_m_s
	);
}

bool Drone::offboard_velocity_goto_nedy(float _north_m, float _east_m, float _down_m, float _yaw_deg, float _speed) {
	promise<void> prom;
	future<void> fut = prom.get_future();
	lock.lock();
	const float callback_frequency = 1000.0f;
	cout << "Attempting to goto nedy coordinate\n";
	telemetry->set_rate_position_velocity_ned(callback_frequency); // 1000 times a sec


	Telemetry::PositionNed final_position { _north_m, _east_m, _down_m } ;
	Offboard::Result result = Offboard::Result::Success;
	float initial_remainder_magnitude = 0.0f;

	Telemetry::PositionVelocityNed start_position_velocity_ned = telemetry->position_velocity_ned();

	std::tuple<float, float, float> start_position_tuple {
		start_position_velocity_ned.position.north_m,
		start_position_velocity_ned.position.east_m,
		start_position_velocity_ned.position.down_m
	};

	std::tuple<float, float, float> end_position_tuple {
		_north_m,
		_east_m,
		_down_m
	};
	const float minimum_distance = 0.5f;

	std::tuple<float, float, float> position_difference_tuple = end_position_tuple - start_position_tuple;
	float maximum_distance = calculate_magnitude(position_difference_tuple);

	const float refresh_rate_speed = maximum_distance * callback_frequency * .0009;
	const float maximum_speed = ( refresh_rate_speed < _speed ) ? refresh_rate_speed : _speed;

	float current_speed = ( ( maximum_speed < _speed ) ? maximum_speed : _speed );
	float prev_magnitude = current_speed;
	float next_magnitude = maximum_distance / 2.0f;

	float new_magnitude = 1000.0f;
	// Gives position and velocity in ned frame.
	telemetry->subscribe_position_velocity_ned( [&](Telemetry::PositionVelocityNed _position_velocity_ned){
		Telemetry::PositionNed _position = _position_velocity_ned.position;
		Telemetry::VelocityNed _velocity = _position_velocity_ned.velocity;

		cout << "heading to : " << final_position << "\n";
		std::tuple new_position_tuple {
			_position.north_m,
			_position.east_m,
			_position.down_m
		};

		std::tuple velocity_tuple {
			_velocity.north_m_s,
			_velocity.east_m_s,
			_velocity.down_m_s
		};

		float velocity_tuple_magnitude = calculate_magnitude(velocity_tuple);

		std::tuple diff = end_position_tuple - new_position_tuple;
		new_magnitude = calculate_magnitude(diff);
		cout << "distance = " << new_magnitude << "m\n";
		cout << "current speed: " << velocity_tuple_magnitude << "m/s\n";
		cout << "maximum speed: " << maximum_speed << "m/s\n";
		cout << "new_magnitude: " << new_magnitude << "m\n";
		cout << "prev_magnitude: " << prev_magnitude << "m\n";

		if ( new_magnitude < next_magnitude ) {
			prev_magnitude = next_magnitude;
			next_magnitude *= 0.5f;
			current_speed *= 0.5f;

			if ( current_speed < 1.0f ) {
				current_speed = 1.0f;
			}

			if ( new_magnitude < 0.5f ) {
				next_magnitude = 0.5f;
				prev_magnitude = 1.0f;
				current_speed = 1.0f;
			}
		} else if ( prev_magnitude < new_magnitude && current_speed < maximum_speed ) {
			next_magnitude = prev_magnitude;
			prev_magnitude *= 2.0f;
			current_speed *= 2.0f;
		}

		cout << "setting speed to: " << current_speed << "\n";
		std::tuple new_v = new_velocity(new_position_tuple, end_position_tuple, current_speed);
		cout << "--------------------------------------------------\n";

		result = offboard->set_velocity_ned({
			std::get<0>(new_v),
			std::get<1>(new_v),
			std::get<2>(new_v),
			_yaw_deg
		});
		if ( result != Offboard::Result::Success ) {
			cout << "offboard set position ned failed: result: " << result << endl;
			return;
		}
		if ( new_magnitude < minimum_distance || (!lock.is_locked()) ) {
			cout << "Reached nedy coordinate!\n";
			telemetry->subscribe_position_velocity_ned(nullptr);
			lock.unlock();
			prom.set_value();
		}
		return;
	});

	cout << "Waiting to finish goto\n";
	fut.wait();

	if ( new_magnitude < minimum_distance )
		return true;

	return false;
}

// bool Drone::goto_frdy_coordinate(float _forward_m, float _right_m, float _down_m, float _yaw_deg, float speed) {
// 	
// 	attitude_euler_angle = telemetry->attitude_euler();
// 	
// 	float front_to_north_yaw_deg = -1 * attitude_euler_angle.yaw_deg;
// 
// 	float north_m = -1.0f *_right_m * sin( front_to_north_yaw_deg ) + _forward_m * cos ( front_to_north_yaw_deg );
// 	float east_m = _right_m * cos( front_to_north_yaw_deg ) + _forward_m * sin( front_to_north_yaw_deg );
// 
// 	cout << "(forward_m, right_m, down_m, yaw_deg) to (east_m, north_m, down_m, yaw_deg).\n"
// 		<< "(" << _forward_m << "," << _right_m << "," << _down_m << "," << _yaw_deg << ") = "
// 		<< "(" << north_m << "," << east_m << "," << _down_m << "," << front_to_north_yaw_deg << ")\n";
// 
// 	// Conversion to north, east, down, yaw is necessary.
// 	// goto_nedy_coordinate(north_m, east_m, _down_m, _yaw_deg, speed);
// 	action_goto_nedy_coordinate(north_m, east_m, _down_m, _yaw_deg, speed);
// 	return true;
// }
void Drone::unlock() {
	lock.unlock();
}

bool Drone::takeoff() {
	Action::Result result = action->takeoff();
	if ( result != Action::Result::Success ) return false;
	return true;
}

bool Drone::land() {
	Action::Result result = action->land();
	if ( result != Action::Result::Success ) return false;
	return true;
}
