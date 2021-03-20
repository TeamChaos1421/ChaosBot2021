//setting slow and fast speeds
double speedFast = .75;
double speedSlow = .65;

bool init = true;

double smartDashboardTest = 0.0;
const int avgLength = 5000;

frc::BuiltInAccelerometer accelerometer{};
frc::Timer timer{};

//////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////DEADBAND FUNCTION/////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

float joystickLinearScaledDeadband(const float value) {
  const float deadbandCutoff = 0.08f;

  if (fabs(value) < deadbandCutoff)
  {
    return 0;
  }
  else
  {
    return (value - (fabs(value) / value) * deadbandCutoff) / (1.0 - deadbandCutoff);
  }
}