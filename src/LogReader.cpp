#include "LogReader.hpp"
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
		std::cout<<"Exception Ocurred!\n"<<msg<<std::endl;
		std::cout<<"Exiting Program"<<std::endl;
		exit(1);
	}
};

Log getOdometryLog(std::stringstream &ss)
{
	double ts,x,y,theta;
	if (!(ss>>x>>y>>theta>>ts))
		throw "Proper Laser log was not provided";
	return Log(LogType::ODOM, x, y, theta, {}, ts);
}

Log getLaserLog(std::stringstream &ss)
{
	std::vector<int> data(180);
	double ts,x,y,theta;
	int val;

	if (!(ss>>x>>y>>theta))
		throw "Proper Laser log was not provided";

	for (int i = 0; i < 180; i++)
	{
		if (!(ss>>val))
			throw "Proper Laser log was not provided";
		else
			data[i] = val;
	}
	return Log(LogType::ODOM, x, y, theta, std::move(data), ts);
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
		std::cout<<"Exception Ocurred!\n"<<msg<<std::endl;
		std::cout<<"Exiting Program"<<std::endl;
		exit(1);
	}
	return boost::optional<Log>();
}
