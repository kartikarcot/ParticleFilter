#include "LogReader.hpp"
#include "spdlog/spdlog.h"
#include <sstream>

LogReader::LogReader(const std::string &fName) : fsm(fName)
{
	try
	{
		if (fsm.fail())
			throw "Valid Log file not provided";
	}
	catch (const char *msg)
	{
		SPDLOG_ERROR("{}",msg);
		exit(1);
	}
};

Log getOdometryLog(std::stringstream &ss)
{
	double ts,x,y,theta;

	if (!(ss>>x>>y>>theta>>ts))
		throw "Proper Laser log was not provided";

	return Log(LogType::ODOM, x, y, theta, -1, -1, -1, ts, {});
}

Log getLaserLog(std::stringstream &ss)
{
	std::vector<int> data(LASER_SIZE);
	double ts,x,y,theta,xl,yl,thetal;
	int val;

	if (!(ss>>x>>y>>theta>>xl>>yl>>thetal))
		throw "Proper Laser log was not provided";

	for (int i = 0; i < LASER_SIZE; i++)
	{
		if (!(ss>>val))
			throw "Proper Laser log was not provided";
		else
			data[i] = val;
	}

	if (!(ss>>ts))
		throw "Timestamp not provided in laser log";

	return Log(LogType::LASER, x, y, theta, xl, yl, thetal, ts, data);
}

boost::optional<Log> LogReader::getLog()
{
	std::string line;
	std::string word;
	std::stringstream ss;
	ss.clear();
	try
	{
		if (!std::getline(fsm, line))
			return boost::optional<Log>();
		ss<<line;
		if(!(ss>>word))
			throw "Proper log record was not provided";
		else
		{
				if(word == "O")
					return getOdometryLog(ss);
				else if (word == "L")
					return getLaserLog(ss);
				else
					throw "Proper prefix for log was not provided";
		}
	}
	catch (const char *msg)
	{
		SPDLOG_ERROR("{}",msg);
		exit(1);
	}
	return boost::optional<Log>();
}
