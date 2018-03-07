//#include "Logger.h"
#include "ros/ros.h"
#include <iostream>
#include <Logger.h>
#include <unistd.h>

int main(int argc, char *argv[]){

	ros::init(argc, argv, "msl_logger_test");

	std::cout << "\n\n\n";
	std::cout << "A test for the Carpe Noctem logger!\n\n";

	msl::Logger* l1 = msl::Logger::getInstance();
    msl::Logger* l2 = msl::Logger::getInstance();

    std::cout << "\n\nL1: " << l1 << "\nL2: " << l2 << '\n';
    //l1->setFileName("test");
    //l1->setFileLvl(msl::LogLevels::debug);
    l1->enableLogging();

    std::cout << "\n\nUTC TIME       ALICA TIME     ID LVL     MESSAGE\n";
    l2->log("NONE", "test Nummer 1 von l2");
    l1->log("NONE", "test Nummer 2 von l1\n");

    usleep(512);
    l1->log("NONE", "test Nummer 3 von l1\n");
    l2->log("NONE", "test Nummer 4 von l2");

    l1->log("NONE", "Test als info",msl::LogLevels::info);

    std::cout << "Setze threshold auf errors.\n";
    //l2->setLvlThreshold(msl::LogLevels::error);
    l1->log("NONE", "Test als warn ohne explizite Ausgabe auf Console!",msl::LogLevels::warn);
    l1->log("NONE", "Test als warn mit expliziter Ausgabe auf Console!",msl::LogLevels::warn,true);

    l1->log("Hallo", "test mit " + std::to_string(2));

    //l1->log2C("ein test nur auf der Console");

    std::cout << "Und jetzt der Langzeit-Test:\n";

    //l2->setLvlThreshold(msl::LogLevels::info);

    for(int i = 0; i < 100; i++){

    	l1->log("STILL_NONE", "Anzahl der durchlaeufe: " + std::to_string(i),msl::LogLevels::console);
    	usleep(30);
    	l1->log("STILL_NONE", "Anzahl der durchlaeufe: " + std::to_string(i),msl::LogLevels::console);
    	usleep(30);
    	l1->log("STILL_NONE", "Anzahl der durchlaeufe: " + std::to_string(i),msl::LogLevels::console);
    	usleep(30);
    	l1->log("STILL_NONE", "Anzahl der durchlaeufe: " + std::to_string(i),msl::LogLevels::console);
    	usleep(30);
    }

    ros::shutdown();
    return 0;
}
