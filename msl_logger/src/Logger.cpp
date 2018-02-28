#include "Logger.h"
#include "MSLWorldModel.h"
#include "SystemConfig.h"
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <time.h>
#include <unistd.h>

namespace msl
{
	Logger::Logger(MSLWorldModel *wm)
	{
		this->wm = wm;
		supplementary::SystemConfig *sc = supplementary::SystemConfig::getInstance();
		this->location = sc->getLogPath();
		this->enabled = (*sc)["Behaviour"]->get<bool>("BehaviourLogging.enableLogging", NULL);
		this->enableConsole = (*sc)["Behaviour"]->get<bool>("BehaviourLogging.enableConsole", NULL);
		this->thresholdLvl = (*sc)["Behaviour"]->get<int>("BehaviourLogging.debugLevel", NULL);
		if(this->thresholdLvl == LogLevels::console){
			this->initConsole = true;
		}else{
			this->initConsole = false;
			this->setFileName("BehaviourLog");
		}
	}

	Logger *Logger::getInstance()
	{
		static Logger instance(msl::MSLWorldModel::get());
		return &instance;
	}

	void Logger::enableLogging()
	{
		this->enabled = true;
	}

	void Logger::disableLogging(){
		this->enabled = false;
	}

	void Logger::setFileName(std::string path)
	{
		std::stringstream filename;

		time_t stamp;
		tm *nun;
		stamp = time(0);
		nun = localtime(&stamp);

		int year = nun->tm_year + 1900;
		int month = nun->tm_mon + 1;
		int day = nun->tm_mday;

		filename << location << '/';
		filename << path << year << '_';
		filename << std::setfill('0') << std::setw(2) << month << '_';
		filename << std::setfill('0') << std::setw(2) << day << '-';
		filename << getTimeStamp(false);
		filename << ".log";

		this->path = filename.str();

		this->outfile.open(this->path.c_str(), std::ios::out);

		if (!this->outfile.good()) // Check for errors
		{
			std::cerr << "specified filename could not be opened for writing!\n";
		}
	}

	int Logger::getLogLevel(){
		return this->thresholdLvl;
	}

	/*void Logger::setLvlThreshold(LogLevels level)
	{
		this->thresholdLvl = level;
		// some logic may be added later
	}*/

	void Logger::log(std::string behaviour, std::string message, LogLevels level, bool forceConsole)
	{
		std::lock_guard<std::mutex> lock(logMutex);
		if (this->enabled)
		{
			std::stringstream msg;

			msg << '[' << getTimeStamp(false) << "] ";

			msg << '[' << wm->getOwnId() << "] ";

			switch (level)
			{
			case LogLevels::console:
				msg << "[CONSOLE]";
				break;
			case LogLevels::debug:
				msg << "[DEBUG]";
				break;
			case LogLevels::info:
				msg << "[INFO ]";
				break;
			case LogLevels::warn:
				msg << "[WARN ]";
				break;
			case LogLevels::error:
				msg << "[ERROR]";
				break;
			}

			msg << ' ' << behaviour << ':' << message;

			//if (*message.end() != '\n')				// Did not work as expected...
			if(message[message.length() - 1] != '\n')	// If the last char in the message is \n, no \n will be added.
			{											// Goal is to eliminate empty lines in the log file.
				msg << '\n';
			}

			if(this->thresholdLvl != LogLevels::console){
				this->outfile << msg.str();
			}

			if(this->enableConsole && level >= this->thresholdLvl || forceConsole && this->enableConsole){
				std::cout << msg.str();
			}
		}
	}

	Logger::~Logger()
	{
		if(!this->initConsole){
			this->outfile.close();
			if (this->outfile.bad())
			{
				std::cerr << "[ERROR] logfile could not be closed!\n";
			}
		}
	}

	std::string Logger::getTimeStamp(bool aTime)
	{
		std::chrono::milliseconds time;
		if(aTime){
			time = std::chrono::milliseconds(this->wm->getTime());
		}else{
			time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
		}

		std::chrono::hours h = std::chrono::duration_cast<std::chrono::hours>(time % std::chrono::hours(24));
		std::chrono::minutes m = std::chrono::duration_cast<std::chrono::minutes>(time % std::chrono::hours(1));
		std::chrono::seconds s = std::chrono::duration_cast<std::chrono::seconds>(time % std::chrono::minutes(1));
		std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(time % std::chrono::seconds(1));

		std::stringstream tmp;
		tmp << std::setfill('0') << std::setw(2) << h.count() << ':';
		tmp << std::setfill('0') << std::setw(2) << m.count() << ':';
		tmp << std::setfill('0') << std::setw(2) << s.count() << '.';
		tmp << std::setfill('0') << std::setw(4) << ms.count();

		return tmp.str();
	}

	void Logger::consoleState(bool state)
	{
		this->enableConsole = state;
	}

	/*void Logger::log2C(std::string message, LogLevels level)
	{
		if(this->enabled && this->enableConsole)
		{
			std::stringstream msg;

			msg << '[' << getTimeStamp(false) << ']';

			msg << '[' << getTimeStamp(true) << ']';

			msg << '[' << wm->getOwnId() << ']';

			switch (level)
			{
			case LogLevels::debug:
				msg << "[DEBUG]";
				break;
			case LogLevels::info:
				msg << "[INFO ]";
				break;
			case LogLevels::warn:
				msg << "[WARN ]";
				break;
			case LogLevels::error:
				msg << "[ERROR]";
				break;
			}

			msg << ' ' << message;

			//if (*message.end() != '\n')				// Did not work as expected...
			if(message[message.length() - 1] != '\n')	// If the last char in the message is \n, no \n will be added.
			{											// Goal is to eliminate empty lines in the log file.
				msg << '\n';
			}
			std::cout << msg.str();
		}
	}*/
}
