#pragma once
#include <fstream>
#include <string>
#include <mutex>
/** \file This file is supposed to create logger that can be used for debugging of every part of the framework uniformly?
 *	Logs haben folgende Form:
 *	[hh:mm:ss.uu][ROBOTID???][LEVEL]MESSAGE
 *	h     == Stunde
 *	m     == Minute
 *	s     == Sekunde
 *	u     == Milisekunde
 *	LEVEL == LOGLEVEL
 *
 *	Dateiname hat folgende Form:
 *	jjjj_MM_dd-hh:mm:ss.uuuu.log
 *	j	  == Jahr
 *	M	  == Monat
 *	d	  == Tag
 *	h	  == Stunde
 *	m	  == Minute
 *	s	  == Sekunde
 *	u     == Milisekunde
 *
 *
 *
 */
/// TODO Werte aus config datei holen
/// TODO behavior info in logdatei
namespace msl
{
	class MSLWorldModel;

	enum LogLevels
	{
		console,/// aufduehrlichste Stufe. Findet nur in der Console statt. Nur fuer config datei!!!!
		debug,	/// aufduehrlichste Stufe. Ausgaben werden auch in datei geschrieben.
		info,	/// weniger ausfuehrlich als debug.
		warn, 	/// leichte Fehler, die im Normalfall nicht passieren sollten.
		error	/// nur schwere Fehler.
	};
	class Logger
	{
	  public:

		/** @fn getInstance()
		 *  @brief This method will return a pointer to the central logger instance.
		 */
		static Logger *getInstance(); // Meyers' Singleton --- Aus Referenzbuch

		/** @fn setLvlThreshold(LogLevels level)
		 *  @brief This method does set the default threshold for output to the console.
		 *         It does temporaryly overwirte the system default.(Originally debug)
		 *         Logs can be written to console explicitely with log2c() or by setting
		 *         the forceConsole in the log method.
		 *
		 *  @param level The loglevel that schould be used
		 *
		 */
		//void setLvlThreshold(LogLevels level);

		/** @fn enableLogging()
		 *  @brief This Method is supposed to enable the debugging functions explicitly,
		 *         temporaryly overwriting the system settings.
		 *
		 */
		void enableLogging();

		/** @fn disableLogging()
		 *  @brief This method is dupposed to disable the logging explicitly, temporarily
		 *         overwriting the system settings.
		 *
		 */
		void disableLogging();

		/** @fn consoleState(bool state)
		 *  @brief This method is used to enable or disable the console output explicitly.
		 */
		/// TODO Config wert!!!
		void consoleState(bool state); // console output allowed == true ... disallowed == false


		/** @fn log(std::string message)
		 *
		 *	@brief This method writes a message to a central log file.
		 *
		 *	@param message The log message that will be written to the file
		 *		   If left empty there will be an empy timestamp.
		 *		   Leaving this empty does not break anything, but defeats the purpose of logging.
		 *
		 *	@param level The level that is used to log.
		 *	       Defaults to debug.
		 *
		 *	@param forceConsole Flag to signal that the message should always be written to
		 *	       the console, if console logging is enabled.
		 *	       Defaults to false.
		 */
		void log(std::string behaviour, std::string message, LogLevels level = LogLevels::debug, bool forceConsole = false);  	// logs a message

		/** @fn log2C(std::string message, LogLevels level)
		 *
		 *  @brief Only outputs to console.
		 *
		 *  @param message The log message that will be written to the file
		 *		   If left empty there will be an empy timestamp.
		 *		   Leaving this empty does not break anything, but defeats the purpose of logging.
		 *
		 *	@param level The level that is used to log.
		 *	       Defaults to debug.
		 *
		 */
		//void log2C(std::string message, LogLevels level = LogLevels::debug);							// Explizite Ausgabe auf der Konsole.

		int getLogLevel();

		~Logger();							// Destruktor schließt ausgabestream und giebt eventuelle resourcen frei.

	  private:

		int thresholdLvl;					// Threshold at which level console output starts
		bool enabled;						// flag that shows if logging is active or not
		bool enableConsole;
		bool initConsole;
		std::string location;				// the path read from config file
		std::string path;					// the path with generated file name
		std::ofstream outfile;				// output where the logs are written to

		MSLWorldModel* wm;					// Pointer to the worldmodel for meta informations
											// The following things are used:
											//     -> alicaTime ( wm->getTime()  )
											//     -> robotId   ( wm->getOwnId() )
		std::mutex logMutex;

		std::string getTimeStamp(bool aTime);// otputs the current time based on the alicaTime
		void setFileName(std::string path); /// unnoetig, da dateiname in config.
		Logger(MSLWorldModel* wm);      	 // aus Referenzbuch  ---  private konstruktor

		// Copyconstructor and assignment operator could be overwritten but apparently there's no need to...
		//Logger(const &Logger); 			// aus Referenzbuch
		//Logger& operator = const Logger&; // aus Referenzbuch
	};
}
