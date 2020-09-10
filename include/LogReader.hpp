#ifndef LOGREADER_H
#define LOGREADER_H

#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <boost/optional.hpp>
#include <ParticleFilter.hpp>

enum LogType {ODOM, LASER};
#define LASER_SIZE 180

struct Log
{
	public:
		LogType logType;
		std::vector<int> laserdata;
		Pose2D robotPose; 
		double timestamp;
		Pose2D laserPose;
		Log(const LogType &recordType,
				const double &_x,
				const double &_y,
				const double &_theta,
				const double &_xl,
				const double &_yl,
				const double &_thetal,
				const double &_ts,
				const std::vector<int> &_laser) :
			logType(recordType),
			robotPose(_x,_y,_theta),
			laserPose(_xl,_yl,_thetal),
			timestamp(_ts),
			laserdata(_laser)
		{};
		Log() = delete;
};


class LogReader
{
	public:
		std::fstream fsm;
		LogReader(const std::string &fName);
		// do not want to deal with copying and assignment. Lets make it singleton
		LogReader(const LogReader &rhs) = delete;
		LogReader& operator= (const LogReader &rhs) = delete;
		// we wish to retain move constructor. with default semantics.
		boost::optional<Log> getLog();
};

#endif
