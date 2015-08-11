#pragma once

struct LinePoint{
        double x;
        double y;
};

struct Particle {
        double posx;
        double posy;
        double heading;
        double weight;
};

struct Position {
        double x;
        double y;
        double heading;
};

struct RobotVelocity {
        double vx;
        double vy;
        double w;
};


struct MovingRobot {
        Position position;
        RobotVelocity velocity;
};
