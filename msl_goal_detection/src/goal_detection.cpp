//
//#include "goal_detection.h"
//
//struct position_msg {
//	double x;
//	double y;
//	double theta;
//	double certainty;
//};
//
//goal_detection_config config = {
//	.back_width = 2.12f, /* in meter, innere torbreite */
//	.goal_depth = 0.52f, /* in meter, tortiefe */
//	.back_width_tolerance = 0.03f, /* in meter, torbreiten-toleranz */
//	.maximums_angle_distance = 25,
//	.view_area_angle = degree_to_rad(130.0f),
//	.min_distance = 0.1f,
//	.max_distance = 4.0f,
//	.reduction_factor = 5.0f
//};
//
///**
// * @brief The main function of the goal detection
// *
// * @param argc Argument count
// * @param argv Argument vector
// *
// * @return 0 if successful.
// */
//int main(int argc, char *argv[])
//{
//	if(argc < 3)
//	{
//		print_help(config);
//		return -1;
//	}
//	if(!parse_command_line_args(argc, argv, config))
//	{
//		cout << "Failed to parse command line parameters." << endl;
//		print_help(config);
//		return -1;
//	}
//
//	ros::init(argc, argv, node_name);
//	ros::NodeHandle n;
//	publisher = n.advertise<PositionInfo>(publisher_topic, 1000);
//	publisher_filtered_max = n.advertise<geometry_msgs::PointStamped>("/goal/filter", 1000);
//	ros::Subscriber sub = n.subscribe(scanner_topic, 1000, scan_message_recieved);
//
//	ros::spin();
//	return 0;
//}
//
///**
// * @brief Uses the command line parameters to set values on a goal_detection_config.
// *
// * @param argc Argument count
// * @param argv Argument vector
// * @param dest Destination config struct
// * @return true if successful.
// */
//bool parse_command_line_args(int argc, char *argv[], goal_detection_config &dest)
//{
//	if(argc < 3) return false;
//	// argv[0] is the executable path
//	dest.back_width = atof(argv[1]);
//	dest.goal_depth = atof(argv[2]);
//
//	// using fall-through to parse command line parameters
//	switch(argc)
//	{
//		case 8: dest.reduction_factor = atof(argv[7]);
//		case 7: dest.max_distance = atof(argv[6]);
//		case 6: dest.min_distance = atof(argv[5]);
//		case 5: dest.maximums_angle_distance = atoi(argv[4]);
//		case 4: dest.back_width_tolerance = atof(argv[3]);
//		case 3: return true;
//		default: return false;
//	}
//}
//
///**
// * @brief Prints the help with given default config values.
// *
// * @param default_config The default values to print
// */
//void print_help(const goal_detection_config default_config)
//{
//	cout
//		<< "Help" << endl
//		<< "./" << node_name
//		<< "[back-width]" << " "
//		<< "[goal-depth]" << " "
//		<< "<back-width-tolerance>" << " "
//		<< "<maximums-angle-distance>" << " "
//		<< "<min-point-distance>" << " "
//		<< "<max-point-distance>" << " "
//		<< "<reduction-factor>" << " "
//		<< endl	;
//	cout << endl;
//	cout
//		<< "\t" << "[back-width]" << "\t"
//			<< "Width of the goal back plane. In meters." << endl
//		<< "\t" << "[goal-depth]" << "\t"
//			<< "Depth of the goal. Distance from post to back plane. In meters." << endl
//		<< "\t" << "<back-width-tolerance>" << "\t"
//			<< "Back plane detection tolerance. In meters." << " [Default: " << default_config.back_width_tolerance << "]" << endl
//		<< "\t" << "<maximums-angle-distance>" << "\t"
//			<< "Distance between two found maximums. In 'units'. " << " [Default: " << default_config.maximums_angle_distance << "]" << endl
//		<< "\t" << "<min-point-distance>" << "\t"
//			<< "Distance to scanner that a maximum should have at least. In meters. " << " [Default: " << default_config.min_distance << "]" << endl
//		<< "\t" << "<max-point-distance>" << "\t"
//			<< "Distance to scanner that a maximum should have at most. In meters. " << " [Default: " << default_config.max_distance << "]" << endl
//		<< "\t" << "<reduction-factor>" << "\t" << endl
//			<< "Factor to reduce the amount of data by taking averages." << " [Default: " << default_config.reduction_factor << "]" << endl
//	;
//}
//
///**
// * @brief Gets executed as soon as the laser scanner publishes a new message on the /scan topic.
// *
// * @param msg The message of the scanner. Contains point data.
// */
//void scan_message_recieved(LaserScan::ConstPtr msg)
//{
//	// reduce points to flatten the points by averaging some of them out
//	vector<float> reduced = reduce_points(msg->ranges, (int)config.reduction_factor);
//	// cout << "all count: " << msg->ranges.size() << endl;
//
//	// find maximum values of these points
//	vector<pair<int, float>> maximums = find_maximums(msg->ranges, config.maximums_angle_distance);
//	// cout << "Maximum count: " << maximums.size() << endl;
//
//	// filter out measurement errors
//	vector<pair<int, float>> okay_points = filter_points(maximums, msg->angle_min, msg->angle_increment);
//	// cout << "okay_points count: " << okay_points.size() << endl;
//
//	// convert maximums to cartesian coordinates to find back plane candidates later
//	vector<Vector3> points = polar_to_cartesian(okay_points, msg->angle_min, msg->angle_increment);
//	// cout << "points count: " << points.size() << endl;
//
///*
//	for (auto &point : points)
//	{
//		cout << "x: " << point.getX() << "\t" << "y: " << point.getY() << endl;
//	}
//	cout << "Looking for back." << endl;
//*/
//
//	// Set precision for debugging purposes
//	cout.precision(3);
//
//	// find candidates for the goal corners by maximums
//	vector<pair<Vector3, Vector3>> corner_candidates = find_back_candidates(points, config.back_width, config.back_width_tolerance);
//
//	if(corner_candidates.size() > 0)
//	{
//		// sort candidates by their distance to the scanner.
//		// the more farther away these two points are, the better
//		std::sort(corner_candidates.begin(), corner_candidates.end(), [](const pair<Vector3, Vector3> &a, const pair<Vector3, Vector3> &b) {
//			float a_len = a.first.length() * a.first.length() + a.second.length() * a.second.length();
//			float b_len = b.first.length() * b.first.length() + b.second.length() * b.second.length();
//			return a_len > b_len;
//		});
//
//		vector<position_msg> positions;
//		for (auto &corner_pair : corner_candidates)
//		{
//			auto p1 = corner_pair.first; // corner 1
//			auto p2 = corner_pair.second; // corner 2
//			auto back_candidate = p1 - p2; // back plane vector
//
//			// get relative angle of the back plane
//			float theta = calculate_angle(y_axis, back_candidate);
//
//			// vector orthogonal to the back plane
//			// has the length of the goal depth
//			auto post_vector = back_candidate.rotate(z_axis, M_PI / 2).normalized() * config.goal_depth;
//
//			auto half_back_candidate = back_candidate.normalized() * (back_candidate.length() / 2.0f);
//
//			// position of the (half) back plane as a vector
//			// this is the center of the back plane
//			auto back_center_absolute =  p2 + half_back_candidate;
//
//			// add the vector of the goal depth to the absolute back plane center
//			auto scanner_center_offset = back_center_absolute + post_vector;
//
//			// add our offset of the scanner position to the goal keeper center
//			auto offset = scanner_center_offset + scanner_offset;
//
//			cout
//				<< "(" << p1.getX() << " | " << p1.getY() << ")"
//				<< " -> "
//				<< "(" << p2.getX() << " | " << p2.getY() << ")"
//				<< "\t"
//				<< "[" << back_candidate.length() << "]"
//				<< "\t"
//				<< "[" << theta << ", " << rad_to_degree(theta) << "]"
//				<< "\t"
//				<< "(" << back_center_absolute.getX() << " | " << back_center_absolute.getY() << ")"
//				<< "(" << offset.getX() << " | " << offset.getY() << ")"
//				<< endl;
//
//			float scanner_center_offset_length = (float)scanner_center_offset.length();
//
//			// check if the calculated goal position is extremely far away.
//			// If so, this is point extremely wrong.
//			// uses the back plane width as a reference value.. just because.
//			if(scanner_center_offset_length > config.back_width)
//				continue;
//
//			// Add this goal center to the potential points list.
//			position_msg le = {
//				.x = offset.getX(),
//				.y = offset.getY(),
//				.theta = theta,
//				.certainty = scanner_center_offset_length
//			};
//			positions.push_back(le);
//		}
//
//		if(positions.size() > 0)
//		{
//			// just take the first one and publish it.
//			position_msg pos = positions[0];
//			publish_message(publisher, pos.x, pos.y, pos.theta);
//		}
//	}
//}
//
///**
// * @brief Reduces the number of data points on a given vector.
// *
// * @param original Point data to reduce/flatten.
// * @param factor The factor to use.
// * @return The reduce points. The new vector will have the size (original.size() / factor)
// */
//vector<float> reduce_points(vector<float> original, int factor)
//{
//	vector<float> reduced(original.size() / factor);
//	for (size_t i = 0; i < reduced.size(); ++i)
//	{
//		float sum = 0;
//		for (int j = 0; j < factor; ++j)
//		{
//			sum += original[i * factor + j];
//		}
//		sum /= factor;
//		reduced[i] = sum;
//	}
//	return reduced;
//}
//
///**
// * @brief Checks if a new potential maximum has a specific distance to all other found maximums.
// *
// * @param vec Previous found maximums, by index.
// * @param value The index of the potential maximum.
// * @param threshold Distance to the other maximums that the new maximum has to fulfil (by index).
// * @return true if the potential maximum satisfies the requirements.
// */
//bool satisfies_threshold(vector<int> &vec, int value, int threshold)
//{
//	if(vec.size() == 0)
//		return true;
//	for (auto &x : vec)
//	{
//		if (is_in_range(x, value, threshold))
//			return false;
//	}
//	return true;
//}
//
///**
// * @brief Looks for peaks in a given vector<float> as described in the project documentation.
// *
// * @param points The data points to search.
// * @param index_distance The minimum distance between two maximums.
// * @return A vector of tuple of (index, maximum_value) of the maximums found.
// */
//vector<pair<int, float>> find_maximums(vector<float> points, int index_distance)
//{
//	vector<pair<int, float>> points_pairs(points.size());
//	for (size_t x = 0; x < points.size(); ++x)
//		points_pairs[x] = make_pair(x, points[x]);
//
//	std::sort(points_pairs.begin(), points_pairs.end(), [](pair<int, float> left, pair<int, float> right) {
//		return left.second < right.second;
//	});
//
//	vector<int> xValues;
//	for (auto &point : points_pairs)
//	{
//		auto x = point.first;
//		auto y = point.second;
//		if(std::find(xValues.begin(), xValues.end(), x) != xValues.end())
//		{
//			// Ignore if found
//		}
//		else
//		{
//			if(satisfies_threshold(xValues, x, index_distance))
//			{
//				xValues.push_back(x);
//			}
//		}
//	}
//
//	vector<pair<int, float>> result;
//	for (auto &x : xValues)
//		result.push_back(make_pair(x, points[x]));
//	return result;
//}
//
///**
// * @brief Filters data points. Kicks out some of the random measurement errors.
// *
// * @param polars A vector of polar coordinate tuples (index, length).
// * @param start_angle The start angle.
// * @param angle_increment The amount each angle increments per measurement.
// * @return A filtered vector of polar coordinate tuples (index, length).
// */
//vector<pair<int, float>> filter_points(vector<pair<int, float>> polars, float start_angle, float angle_increment)
//{
//	vector<pair<int, float>> dest;
//	for (auto &value : polars)
//	{
//		float angle = start_angle + angle_increment * value.first;
//		float length = value.second;
//
//		if(length < config.min_distance || length > config.max_distance)
//			continue;
//		if(angle > config.view_area_angle || angle < -config.view_area_angle)
//			continue;
//		dest.push_back(value);
//	}
//	return dest;
//}
//
///**
// * @brief Converts polar coordinates to cartesian coordinates
// *
// * @param polars Vector of polar coordinates.
// * @param start_angle The start angle.
// * @param angle_increment The amount each angle increments per measurement.
// * @return A vector of Vector3, where Vector3.{x|y} are cartesian coordinates. Vector3.z is intentionally left 0.
// */
//vector<Vector3> polar_to_cartesian(vector<pair<int, float>> polars, float start_angle, float angle_increment)
//{
//	vector<Vector3> cartesians(polars.size());
//	if(polars.size() <= 0)
//		return cartesians;
///*
//	std::sort(polars.begin(), polars.end(), [](const pair<int, float> &left, const pair<int, float> &right) {
//		return left.first < right.first;
//	});
//*/
//
//	for (size_t i = 0; i < polars.size(); ++i)
//	{
//		auto value = polars[i];
//
//		float angle = start_angle + angle_increment * value.first;
//		float length = value.second;
//
//		float x = length * cos(angle);
//		float y = length * sin(angle);
//
//		cartesians[i] = Vector3(x, y, 0);
//	}
//	return cartesians;
//}
//
///**
// * @brief Tries to find mathching pairs of coordinates which could be the goal corners.
// *
// * @param maximums The maximums to look at.
// * @return A vector of a pair of (Vector3, Vector3) which could be the corners of the goal.
// */
//vector<pair<Vector3, Vector3>> find_back_candidates(vector<Vector3> maximums, float back_width, float back_width_tolerance)
//{
//	vector<pair<Vector3, Vector3>> candidates;
//	if(maximums.size() <= 0)
//		return candidates;
//
//	for (auto &point : maximums)
//	{
//		if(point.getX() < 0 || point.getY() < 0)
//			continue;
//
//		for (auto &other : maximums)
//		{
//			if(other == point)
//				continue;
//
//			Vector3 back_candidate = point - other;
//			float distance = back_candidate.length();
//			bool is_ok = is_in_range(back_width, distance, back_width_tolerance);
//			if (is_ok)
//			{
//				auto pair = make_pair(point, other);
//				candidates.push_back(pair);
//			}
//		}
//	}
//	return candidates;
//}
//
///**
// * @brief Publishes a new goal position.
// *
// * @param pub The publisher to use.
// * @param x X coordinate relative to the goal keeper.
// * @param y Y coordinate relative to the goal keeper.
// * @param angle Angle relative to the goal keeper.
// */
//void publish_message(ros::Publisher pub, float x, float y, float angle)
//{
//	PositionInfo msg;
//	msg->header.frame_id = "laser";
//	msg->header.stamp = ros::Time::now();
//	msg->x = x;
//	msg->y = y;
//	msg->angle = angle;
//	msg->certainty = 0.0;
//	pub.publish(msg);
//}
//
///**
// * @brief Calculates the angle between two vectors.
// *
// * @param a First vector.
// * @param b Second vector.
// *
// * @return The angle between these vector in radians.
// */
//float calculate_angle(Vector3 a, Vector3 b)
//{
//	return acos((a.dot(b)) / (a.length() * b.length()));
//}
//
///**
// * @brief Converts radians to degrees.
// *
// * @param rad A radian angle value.
// * @return A degree angle value.
// */
//float rad_to_degree(float rad)
//{
//	return rad * 180 / M_PI;
//}
//
///**
// * @brief Converts degrees to radians.
// *
// * @param degree A degree angle value.
// * @return A radian angle value.
// */
//float degree_to_rad(float degree)
//{
//	return degree *  M_PI / 180;
//}
//
///**
// * @brief Checks if a value is in a given interval/range based on a reference value.
// * @details [long description]
// *
// * @param compare_to Reference value
// * @param value The value to check
// * @param threshold The maximum distance to the left/right of the reference value.
// * @return true if the value is in the specified range.
// */
//bool is_in_range(float compare_to, float value, float threshold)
//{
//	return (compare_to - threshold) <= value && value <= (compare_to + threshold);
//}
//
///**
// * @brief Checks if a value is in a given interval/range based on a reference value.
// * @details [long description]
// *
// * @param compare_to Reference value
// * @param value The value to check
// * @param threshold The maximum distance to the left/right of the reference value.
// * @return true if the value is in the specified range.
// */
//bool is_in_range(int compare_to, int value, int threshold)
//{
//	return (compare_to - threshold) <= value && value <= (compare_to + threshold);
//}
