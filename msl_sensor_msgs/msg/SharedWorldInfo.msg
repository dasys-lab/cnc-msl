#constants for ball possession status (must be the same as in MSLEnums.h)
uint8 HAVE_BALL = 0
uint8 LIGHT_BARRIER_UNBLOCKED = 1
uint8 NOT_IN_KICKER_DISTANCE = 2
uint8 ASIDE_OF_KICKER = 3
uint8 NO_BALL_SEEN = 4

int32 senderID
bool participating
bool ownTeamIsMagenta
bool ownGoalIsYellow
uint8 ballPossessionStatus

CorrectedOdometryInfo odom

BallInfo ball
BallInfo negotiatedBall
SharedBallInfo sharedBall

msl_msgs/Point2dInfo[] path
msl_msgs/Point2dInfo[] mergedOpponents
msl_msgs/Point2dInfo[] mergedTeamMembers
msl_msgs/Point2dInfo[] obstacles

#bool ballIsFree
bool ballInPossession

