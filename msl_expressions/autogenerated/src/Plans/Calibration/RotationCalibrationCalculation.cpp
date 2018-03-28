using namespace std;
#include "Plans/Calibration/RotationCalibrationCalculation.h"

/*PROTECTED REGION ID(inccpp1475074396562) ENABLED START*/ //Add additional includes here
#include <regex>
#include <sstream>
#include <MSLWorldModel.h>
#include <SystemConfig.h>
#include <FileSystem.h>
#include <container/CNPoint2D.h>
#include "ConsoleCommandHelper.h"
//#include "Plans/Calibration/RotateOnce.h"

/// NOTE Braucht man das überhaupt???

/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1475074396562) ENABLED START*/ //initialise static variables here
    int RotationCalibrationCalculation::usableValues = 0;
    double RotationCalibrationCalculation::usableValueSum = 0;

    /*PROTECTED REGION END*/
    RotationCalibrationCalculation::RotationCalibrationCalculation() :
            DomainBehaviour("RotationCalibrationCalculation")
    {
        /*PROTECTED REGION ID(con1475074396562) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    RotationCalibrationCalculation::~RotationCalibrationCalculation()
    {
        /*PROTECTED REGION ID(dcon1475074396562) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void RotationCalibrationCalculation::run(void* msg)
    {
        /*PROTECTED REGION ID(run1475074396562) ENABLED START*/ //Add additional options here
        /*if (this->isSuccess() || this->isFailure())
         {
         return;
         }
         //		if (RotateOnce::measurements[0] == NULL)
         {
         RotateOnce::measurements[0] = RotateOnce::measurements[1];
         RotateOnce::measurements[1] = new geometry::CNPoint2D(RotateOnce::measurements[0]->x + 10, 0); // TODO x field
         this->setFailure(true);
         wm->setRobotRadius(RotateOnce::measurements[1]->x);
         cout << "RCC: Erstes Mal" << endl;
         }
         else
         {
         cout << "Measurement[1] =  x:" << RotateOnce::measurements[1]->x << " y: " << RotateOnce::measurements[1]->y
         << endl;
         if (abs(RotateOnce::measurements[1]->y) < ERROR_THRESHOLD)
         {
         usableValues++;
         usableValueSum += RotateOnce::measurements[1]->x;

         cout << "Gute Messung Nr. " << usableValues << ": RobotRadius = " << RotateOnce::measurements[1]->x << endl;

         if (usableValues >= NEEDED_NUM_OF_VALUES)
         {
         double averageRobotRadius = usableValueSum / usableValues;
         cout << "RCC: Finaler RobotRadius: " << averageRobotRadius << endl;
         wm->setRobotRadius(averageRobotRadius);
         this->setSuccess(true);
         }
         else
         {
         calculateNextValue();
         }
         }
         else
         {
         cout << "RCC: Fehler " << RotateOnce::measurements[1]->y << " ist noch nicht gut genug" << endl;
         calculateNextValue();
         }
         }
         /*PROTECTED REGION END*/
    }
    void RotationCalibrationCalculation::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1475074396562) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1475074396562) ENABLED START*/ //Add additional methods here
    void RotationCalibrationCalculation::calculateNextValue()
    {
        // call gnuplot for maximum convenience!!
        // f(x) = x**3 * a + x**2 * b + x * c + d
        // fit f(x) "RotationCalibration.tsv" using 1:2:3:4 via a,b,c,d

        // TODO use measurements for gnuplot call
        /*stringstream cmd;
         string logfile = supplementary::FileSystem::combinePaths(sc->getLogPath(), "RotationCalibration.log");

         if (RotateOnce::logCounter > RotationCalibrationCalculation::MAX_POINTS_USED_FOR_REGRESSION)
         {
         ifstream infile(logfile);
         string firstLine;
         std::getline(infile, firstLine);
         std::getline(infile, firstLine);
         string fileContent;
         string line;
         while (std::getline(infile, line).eof() == false)
         {
         fileContent += line;
         fileContent += "\n";
         }
         fileContent += line;
         infile.close();
         ofstream ofstr(logfile);
         ofstr << fileContent;
         ofstr.flush();
         ofstr.close();

         new gnuplot stirnmtg

         gnuplot -persist -e "f(x) = a*x; fit f(x) '/home/cn/data.csv' u 1:($0 +1) via a,b; plot '/home/cn/data.csv' , f(x), 0; print sprintf('robotRadius=%f',a)"

         }

         cmd << "gnuplot -persist -e \"f(x) = a*x+b; fit f(x) \\\"";
         cmd << logfile;
         cmd << "\\\" u 1:2 via a,b; plot \\\"";
         cmd << logfile;
         cmd << "\\\", f(x), 0; print sprintf(\\\"robotRadius=%f\\\",-b/a)\" 2>&1";

         cout << cmd.str() << endl;
         string gnuplotReturn = supplementary::ConsoleCommandHelper::exec(cmd.str().c_str());
         // cout << "gnuplotReturn: " << gnuplotReturn << endl;

         /* I like C! */
        // match plotted value
        /*const int max_line_size = 400;
         const int output_size = gnuplotReturn.size() + 1;
         const char *ret = gnuplotReturn.c_str();
         char output[output_size];
         char *line;
         char *lastline;
         char *radiusStr;

         strncpy(output, gnuplotReturn.c_str(), output_size);

         // Get last line
         line = strtok(output, "\n");
         while ((line = strtok(NULL, "\n")) != NULL)
         lastline = line;

         // Get the radius as string
         strtok(lastline, "="); // eats robotRadius
         radiusStr = strtok(NULL, "=");
         if (radiusStr == NULL)
         {
         cerr << "ERROR parsing gnuplot output" << endl;
         this->setSuccess(true); // not really tho
         return;
         }

         // Convert it to double
         double calculatedValue = strtod(radiusStr, NULL);
         if (calculatedValue <= 0 || calculatedValue > 500)
         {
         // Something went wrong during parsing when value = 0
         cerr << "ERROR parsing robot radius OR robot radius not appropriate" << endl;
         cerr << "Calculated Robot Radius: " << calculatedValue << endl;
         this->setSuccess(true); // not really tho
         return;
         }
         geometry::CNPoint2D* temp = RotateOnce::measurements[0];
         RotateOnce::measurements[0] = RotateOnce::measurements[1];
         RotateOnce::measurements[1] = temp;

         RotateOnce::measurements[1]->x = calculatedValue;
         RotateOnce::measurements[1]->y = 0;

         cout << "SUCCESS!" << endl;
         cout << "Calculated Robot Radius: " << calculatedValue << endl;

         wm->setRobotRadius(RotateOnce::measurements[1]->x);
         cout << "OH SHIT IT WORKED!" << endl;
         this->setFailure(true);*/
    }
/*PROTECTED REGION END*/
} /* namespace alica */
