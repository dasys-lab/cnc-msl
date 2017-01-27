/*
 * LaserScanListener.cpp
 *
 *  Created on: Jan 27, 2017
 *      Author:  Lisa Martmann
 */

#include <LaserScanListener.h>
#include <SystemConfig.h>
namespace msl
{
	LaserScanListener::LaserScanListener()
	{
		readConfigParameters();
		publisher = n.advertise<msl_msgs::PositionInfo>(publisher_topic.c_str(), 10);
		publisher_filtered_max = n.advertise<geometry_msgs::PointStamped>(filter_topic.c_str(), 10);
		sub = n.subscribe(scanner_topic.c_str(), 10, &LaserScanListener::onLaserScanReceived, (LaserScanListener*)this);

	}

	LaserScanListener::~LaserScanListener()
	{
	}

	void LaserScanListener::readConfigParameters()
	{
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();

		scanner_topic = (*sc)["GoalDetection"]->get < string > ("GoalDetection.Topics.scanner_topic", NULL);
		publisher_topic = (*sc)["GoalDetection"]->get < string > ("GoalDetection.Topics.publisher_topic", NULL);
		filter_topic = (*sc)["GoalDetection"]->get < string > ("GoalDetection.Topics.filter_topic", NULL);

		back_width = (*sc)["GoalDetection"]->get<double>("GoalDetection.back_width", NULL);
		goal_depth = (*sc)["GoalDetection"]->get<double>("GoalDetection.goal_depth", NULL);
		back_width_tolerance = (*sc)["GoalDetection"]->get<double>("GoalDetection.back_width_tolerance", NULL);
		max_angle_distance = (*sc)["GoalDetection"]->get<double>("GoalDetection.max_angle_distance", NULL);
		view_area_angle = (*sc)["GoalDetection"]->get<double>("GoalDetection.view_area_angle", NULL);
		max_distance = (*sc)["GoalDetection"]->get<double>("GoalDetection.max_distance", NULL);
		min_distance = (*sc)["GoalDetection"]->get<double>("GoalDetection.min_distance", NULL);
		reduction_factor = (*sc)["GoalDetection"]->get<double>("GoalDetection.reduction_factor", NULL);

	}

	void LaserScanListener::onLaserScanReceived(sensor_msgs::LaserScanPtr msg)
	{

		tf::Vector3 scanner_offset(0.2, 0.0, 0); // x-, y-, z-Offset, Scanner zu Torwartmitte

		tf::Vector3 z_axis(0, 0, 1);
		tf::Vector3 y_axis(0, 1, 0);

		// reduce points to flatten the points by averaging some of them out
		vector<double> reduced = reduce_points(msg, reduction_factor);
		// cout << "all count: " << msg->ranges.size() << endl;

		// find maximum values of these points
		vector<pair<int, double>> maximums = find_maximums(msg, max_angle_distance);
		// cout << "Maximum count: " << maximums.size() << endl;
//
//		// filter out measurement errors
		vector<pair<int, double>> okay_points = filter_points(maximums, msg);
//		// cout << "okay_points count: " << okay_points.size() << endl;
//
//		// convert maximums to cartesian coordinates to find back plane candidates later
		vector<tf::Vector3> points = polar_to_cartesian(okay_points, msg);
//		// cout << "points count: " << points.size() << endl;
//
//		// Set precision for debugging purposes
//		cout.precision(3);
//
//		// find candidates for the goal corners by maximums
		vector<pair<tf::Vector3, tf::Vector3>> corner_candidates = find_back_candidates(points, back_width,
																						back_width_tolerance);

		if (corner_candidates.size() > 0)
		{
			// sort candidates by their distance to the scanner.
			// the more farther away these two points are, the better
			std::sort(corner_candidates.begin(), corner_candidates.end(),
						[](pair<tf::Vector3, tf::Vector3> a, pair<tf::Vector3, tf::Vector3> b)
						{
							float a_len = a.first.length() * a.first.length() + a.second.length() * a.second.length();
							float b_len = b.first.length() * b.first.length() + b.second.length() * b.second.length();
							return a_len > b_len;
						});

			//FIXME
			vector<msl_msgs::PositionInfo> positions;
			for (auto &corner_pair : corner_candidates)
			{
				auto p1 = corner_pair.first; // corner 1
				auto p2 = corner_pair.second; // corner 2
				auto back_candidate = p1 - p2; // back plane vector

				// get relative angle of the back plane
				double theta = calculate_angle(y_axis, back_candidate);

				// vector orthogonal to the back plane
				// has the length of the goal depth
				auto post_vector = back_candidate.rotate(z_axis, M_PI / 2).normalized() * goal_depth;

				auto half_back_candidate = back_candidate.normalized() * (back_candidate.length() / 2.0);

				// position of the (half) back plane as a vector
				// this is the center of the back plane
				auto back_center_absolute = p2 + half_back_candidate;

				// add the vector of the goal depth to the absolute back plane center
				auto scanner_center_offset = back_center_absolute + post_vector;

				// add our offset of the scanner position to the goal keeper center
				auto offset = scanner_center_offset + scanner_offset;

				cout << "(" << p1.getX() << " | " << p1.getY() << ")" << " -> " << "(" << p2.getX() << " | "
						<< p2.getY() << ")" << "\t" << "[" << back_candidate.length() << "]" << "\t" << "[" << theta
						<< ", " << rad_to_degree(theta) << "]" << "\t" << "(" << back_center_absolute.getX() << " | "
						<< back_center_absolute.getY() << ")" << "(" << offset.getX() << " | " << offset.getY() << ")"
						<< endl;

				double scanner_center_offset_length = scanner_center_offset.length();

				// check if the calculated goal position is extremely far away.
				// If so, this is point extremely wrong.
				// uses the back plane width as a reference value.. just because.
				if (scanner_center_offset_length > back_width)
					continue;

				// Add this goal center to the potential points list.
				msl_msgs::PositionInfo pi;

				pi.x = offset.getX();
				pi.y = offset.getY();
				pi.angle = theta;
				pi.certainty = scanner_center_offset_length;
//				position_msg le = {.x = offset.getX(), .y = offset.getY(), .theta = theta, .certainty =
//											scanner_center_offset_length};
				positions.push_back(pi);
			}

			if (positions.size() > 0)
			{
				// just take the first one and publish it.
				msl_msgs::PositionInfo pos = positions[0];

//				pos->header.frame_id = "laser";
//				pos->header.stamp = ros::Time::now();

				//FIXME missing header bad?
				publisher.publish(pos);

//				publish_message(publisher, pos.x, pos.y, pos.theta);
			}
		}
	}

	vector<double> LaserScanListener::reduce_points(sensor_msgs::LaserScanPtr msg, double factor)
	{
		vector<double> reduced(msg->ranges.size() / factor);
		for (size_t i = 0; i < reduced.size(); ++i)
		{
			double sum = 0;
			for (int j = 0; j < factor; ++j)
			{
				sum += msg->ranges[i * factor + j];
			}
			sum /= factor;
			reduced[i] = sum;
		}
		return reduced;
	}

	vector<pair<int, double> > LaserScanListener::find_maximums(sensor_msgs::LaserScanPtr msg, double index_distance)
	{
		vector<pair<int, double>> points_pairs(msg->ranges.size());
		for (size_t x = 0; x < msg->ranges.size(); ++x)
		{
			points_pairs[x] = make_pair(x, msg->ranges[x]);
		}
		std::sort(points_pairs.begin(), points_pairs.end(), [](pair<int, double> left, pair<int, double> right)
		{
			return left.second < right.second;
		});

		vector<int> xValues;
		for (auto &point : points_pairs)
		{
			auto x = point.first;
			auto y = point.second;
			if (std::find(xValues.begin(), xValues.end(), x) == xValues.end())
			{
				if (satisfies_threshold(xValues, x, index_distance))
				{
					xValues.push_back(x);
				}
			}
		}
		vector<pair<int, double>> result;
		for (auto &x : xValues)
		{
			result.push_back(make_pair(x, msg->ranges[x]));
		}
		return result;
	}

	bool LaserScanListener::satisfies_threshold(vector<int> vec, int value, int threshold)
	{
		if (vec.size() == 0)
		{
			return true;
		}
		for (auto &x : vec)
		{
			if (is_in_range(x, value, threshold))
			{
				return false;
			}
		}
		return true;
	}

	vector<pair<int, double>> LaserScanListener::filter_points(vector<pair<int, double>> polars,
																sensor_msgs::LaserScanPtr msg)
	{
		vector<pair<int, double>> dest;
		for (auto &value : polars)
		{
			float angle = msg->angle_min + msg->angle_increment * value.first;
			float length = value.second;

			if (length < min_distance || length > max_distance)
				continue;
			if (angle > view_area_angle || angle < -view_area_angle)
				continue;
			dest.push_back(value);
		}
		return dest;
	}

	/**
	 * @brief Converts polar coordinates to cartesian coordinates
	 *
	 * @param polars Vector of polar coordinates.
	 * @param start_angle The start angle.
	 * @param angle_increment The amount each angle increments per measurement.
	 * @return A vector of Vector3, where Vector3.{x|y} are cartesian coordinates. Vector3.z is intentionally left 0.
	 */
	vector<tf::Vector3> LaserScanListener::polar_to_cartesian(vector<pair<int, double>> polars,
																sensor_msgs::LaserScanPtr msg)
	{
		vector<tf::Vector3> cartesians(polars.size());
		if (polars.size() <= 0)
		{
			return cartesians;
		}
		/*
		 std::sort(polars.begin(), polars.end(), [](const pair<int, float> &left, const pair<int, float> &right) {
		 return left.first < right.first;
		 });
		 */

		for (size_t i = 0; i < polars.size(); ++i)
		{
			auto value = polars[i];

			double angle = msg->angle_min + msg->angle_increment * value.first;
			double length = value.second;

			double x = length * cos(angle);
			double y = length * sin(angle);

			cartesians[i] = tf::Vector3(x, y, 0);
		}
		return cartesians;
	}

	//FIXME params??
	vector<pair<tf::Vector3, tf::Vector3>> LaserScanListener::find_back_candidates(vector<tf::Vector3> maximums,
																					double back_width,
																					double back_width_tolerance)
	{
		vector<pair<tf::Vector3, tf::Vector3>> candidates;
		if (maximums.size() <= 0)
			return candidates;

		for (auto &point : maximums)
		{
			if (point.getX() < 0 || point.getY() < 0)
			{
				continue;
			}

			for (auto &other : maximums)
			{
				if (other == point)
				{
					continue;
				}

				tf::Vector3 back_candidate = point - other;
				double distance = back_candidate.length();
				bool is_ok = is_in_range(back_width, distance, back_width_tolerance);
				if (is_ok)
				{
					auto pair = make_pair(point, other);
					candidates.push_back(pair);
				}
			}
		}
		return candidates;
	}

	bool LaserScanListener::is_in_range(double compare_to, double value, double threshold)
	{
		return (compare_to - threshold) <= value && value <= (compare_to + threshold);
	}

	bool LaserScanListener::is_in_range(int compare_to, int value, int threshold)
	{
		return (compare_to - threshold) <= value && value <= (compare_to + threshold);
	}

	double LaserScanListener::calculate_angle(tf::Vector3 a, tf::Vector3 b)
	{
		return acos((a.dot(b)) / (a.length() * b.length()));
	}

	double LaserScanListener::rad_to_degree(double rad)
	{
		return rad * 180 / M_PI;
	}

//FIXME
//	void publish_message(ros::Publisher pub, double x, double y, double angle, double certainty)
//	{
//		msl_msgs::PositionInfo msg;
//		msg->header.frame_id = "laser";
//		msg->header.stamp = ros::Time::now();
//		msg->x = x;
//		msg->y = y;
//		msg->angle = angle;
//		msg->certainty = 0.0;
//		pub.publish(msg);
//	}

}

