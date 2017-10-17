
#define GNUPLOT_ENABLE_PTY
#include "helpers/KeyHelper.h"
#include "helpers/LocalizeDebug.h"
#include "helpers/SpicaHelper.h"
#include "helpers/gnuplot-iostream.h"

#include <supplementary/BroadcastID.h>
#include <msl/robot/IntRobotID.h>
#include <msl/robot/IntRobotIDFactory.h>
#include <SystemConfig.h>

#include "ros/ros.h"

#include <iostream>
#include <math.h>
#include <signal.h>
#include <unistd.h>
#include <vector>

SpicaHelper *sh;

void int_handler(int sig)
{
    int mypid = getpid();
    printf("[%i] Ouch - shot in the ...\n", mypid);
    printf("[%i] exit\n", mypid);
    sh->sendVisionControl((int)'p', 0);
    delete sh;
    ros::shutdown();
    exit(2);
}

int main(int argc, char *argv[])
{
    // Gnuplot gp("gnuplot -persist");
    Gnuplot gp;

    double mx = 0, my = 0, x, y, anglestep, angle;
    int mb = 1;
    LocalizeDebug ldb;
    int n = 1;
    bool plotopen = false;
    sh = new SpicaHelper();
    sh->receiverID = new supplementary::BroadcastID(nullptr, 0);
    sh->initialize("CNVisionDataViewer", false);
    msl::robot::IntRobotIDFactory factory;
    if (argc > 1)
    {
        for (int i = 1; i < argc; i++)
        {
            if (std::string(argv[i]) == "-id")
            {
                if (i + 1 < argc)
                {
                    auto intID = atoi(argv[i + 1]);
                    std::vector<uint8_t> robotId;

                    for (int i = 0; i < sizeof(int); i++)
                    {
                        robotId.push_back(*(((uint8_t *)&intID) + i));
                    }
                    sh->receiverID = factory.create(robotId);
                    i++;
                }
            }
            if (std::string(argv[i]) == "-n")
            {
                if (i + 1 < argc)
                {
                    supplementary::Configuration *globals = (*supplementary::SystemConfig::getInstance())["Globals"];
                    auto intID = globals->get<int>("Globals", "Team", argv[i + 1], "ID", NULL);
                    std::vector<uint8_t> robotId;

                    for (int i = 0; i < sizeof(int); i++)
                    {
                        robotId.push_back(*(((uint8_t *)&intID) + i));
                    }
                    sh->receiverID = factory.create(robotId);
                    cout << "Robot: " << argv[i + 1] << " [" << sh->receiverID << "]" << endl;
                    i++;
                }
            }
        }
    }

    if (dynamic_cast<const supplementary::BroadcastID *>(sh->receiverID) != nullptr)
    {
        cout << "Robot ID: ";
        int intID;
        cin >> intID;
        std::vector<uint8_t> robotId;

        for (int i = 0; i < sizeof(int); i++)
        {
            robotId.push_back(*(((uint8_t *)&intID) + i));
        }
        sh->receiverID = factory.create(robotId);
    }

    int currentKey = EOF;
    signal(SIGINT, int_handler);
    sh->sendVisionControl((int)'P', 0);

    while (ros::ok())
    {
        currentKey = KeyHelper::checkKeyPress();
        if (sh->lpdirty == true)
        {
            std::vector<std::pair<double, double>> xy_pts;
            std::vector<std::pair<double, double>> xy_pts2;
            std::vector<std::pair<double, double>> xy_pts3;
            std::vector<std::pair<double, double>> ball;
            std::vector<std::pair<double, double>> ballConfidence;
            std::vector<std::pair<double, double>> obstacles;

            for (unsigned int i = 0; i < sh->linePoints.size(); i++)
            {
                xy_pts.push_back(std::make_pair(sh->linePoints.at(i).x, sh->linePoints.at(i).y));
            }
            Particle p;
            p.posx = sh->pos.x;
            p.posy = sh->pos.y;
            p.heading = sh->pos.angle;
            p.weight = sh->pos.certainty;

            angle = 0;
            anglestep = (2 * 3.141592) / sh->distanceScan.size();
            for (unsigned int i = 0; i < sh->distanceScan.size(); i++)
            {
                angle += anglestep;
                y = sin(angle) * sh->distanceScan[i];
                x = cos(angle) * sh->distanceScan[i];
                xy_pts2.push_back(std::make_pair(x, y));
            }

            for (unsigned int i = 0; i < sh->obstacles.size(); i++)
            {
                obstacles.push_back(std::make_pair(sh->obstacles[i].x, sh->obstacles[i].y));
            }
            xy_pts3.push_back(std::make_pair(-7990, sh->pos.certainty * 8000));
            ball.push_back(std::make_pair(sh->bi.point.x, sh->bi.point.y));
            ball.push_back(std::make_pair(sh->bi.point.x + sh->bi.velocity.vx, sh->bi.point.y + sh->bi.velocity.vy));

            ballConfidence.push_back(std::make_pair(7990, sh->bi.confidence));
            gp << "set xrange [-8000:8000]\nset yrange [-8000:8000]\n";
            gp << "set size ratio 1\n";
            gp << "plot '-' w points t 'Linepoints'";
            gp << ", '-' t 'Field'";
            if (xy_pts2.size() > 1)
                gp << ", '-' w lines t 'DistanceScan'";
            gp << ", '-' w points t 'Loc Conf'";
            gp << ", '-' w linespoints t 'Ball'";
            gp << ", '-' w points t 'Ball Conf'";
            if (obstacles.size() > 1)
                gp << ", '-' w points t 'Obstacles'\n";
            else
                gp << "\n";
            gp.send(xy_pts);
            gp.send(ldb.drawFieldForParticle(p, 0));
            if (xy_pts2.size() > 1)
                gp.send(xy_pts2);

            gp.send(xy_pts3);
            gp.send(ball);
            gp.send(ballConfidence);
            if (obstacles.size() > 1)
                gp.send(obstacles);

            sh->lpdirty = false;
            plotopen = true;
        }

        usleep(50000);
        // if(plotopen) gp.getMouse(mx, my, mb, "right click to exit.");
        if (currentKey != EOF)
        {
            sh->sendVisionControl(currentKey, 0);
        }

        // printf("You pressed mouse button %d at x=%f y=%f\n", mb, mx, my);
        if (mb == 3)
        {
            delete sh->receiverID;
            return 1;
        }
    }
    sh->sendVisionControl((int)'p', 0);
    delete sh->receiverID;
}
