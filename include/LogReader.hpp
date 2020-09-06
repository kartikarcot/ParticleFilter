#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <boost/optional.hpp>

enum LogType {ODOM, LASER, UNKNOWN};

struct Log
{
	LogType lg;
	const std::vector<int> laserdata;
	const double x, y, theta, timestamp;
	Log(const LogType &recordType,
			const double _x,
			const double _y,
			const double _theta,
			const std::vector<int> &laser,
			const double ts) : 
		lg(recordType),
		x(_x),
		y(_y),
		theta(_theta),
		laserdata(laser),
		timestamp(ts)
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
