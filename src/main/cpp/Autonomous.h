const int avgLength = 5000;

int i;

const int teleopAvgLength = 5;
double teleopDriveAvg[teleopAvgLength];
double teleopTurnAvg[teleopAvgLength];
double teleopDriveAvgSum;
double teleopTurnAvgSum;

double avg[avgLength];
double a[11] = {0,0,0,0,0,0,0,0,0,0,0};
double duration = 2;
double sum;