//setting slow and fast speeds
double speedFast = .75;
double speedSlow = .65;
double driveSpeed=speedFast;
double turn;
bool init = true;
double smartDashboardTest = 0.0;

frc::BuiltInAccelerometer accelerometer{};
frc::Timer timer{};

frc::Timer SBAtimer;
bool shootBallsShort=false;
int movnum = 0;
frc::Timer movtmr;
double lcounts, rcounts, mtime;
frc::ADXRS450_Gyro gyro;
double bearing = 0;
bool linearDone=false;
bool angleDone=false;
frc::Timer LBtimer;

//---------------------ShooterPID Setup------------------------------------------------------
static const int shooterDeviceID = 60;
rev::CANSparkMax m_shooter{shooterDeviceID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANPIDController m_shooterPidController = m_shooter.GetPIDController();
rev::CANEncoder m_shooterEncoder = m_shooter.GetEncoder();
double shooter_kP = 5e-4, shooter_kI = 1e-6, shooter_kD = 0, shooter_kIz = 0, shooter_kFF = 0.000015, shooter_kMaxOutput = 1.0, shooter_kMinOutput = -1.0;
const double shooterMaxRPM = 5700;
//------------------------------------------------------------------------------------------

WPI_TalonFX *m_leftMotor1a = new WPI_TalonFX(23);
WPI_TalonFX *m_leftMotor2a = new WPI_TalonFX(22);
WPI_TalonFX *m_rightMotor1a = new WPI_TalonFX(21);
WPI_TalonFX *m_rightMotor2a = new WPI_TalonFX(20);
frc::SpeedControllerGroup *m_left = new frc::SpeedControllerGroup(*m_leftMotor1a, *m_leftMotor2a);
frc::SpeedControllerGroup *m_right = new frc::SpeedControllerGroup(*m_rightMotor1a, *m_rightMotor2a);
frc::DifferentialDrive *m_robotDrive = new frc::DifferentialDrive(*m_left, *m_right);
frc::XboxController driver{0};
frc::XboxController copilot{1};

int lkPIDLoopIdx = 0;
int rkPIDLoopIdx = 0;
// PID Coefficients
double kP = 0.125, kI = 0, kD = 0, kIz = 0, kFF = 0, kMaxOutput = 0.25, kMinOutput = -0.25;
bool autoFlag = false;
double cntpft = 14500;
double maxSpeed=1;
float angle;
double offset;
float speed;
float absSpeed;
//ControlPanel CP;
//ControlPanel CM;
bool PositionControlDone;
bool RotationControlDone;

//char turns;
std::string gameData;
TalonSRX m_feeder = {2};
TalonSRX climb1 = {7};
TalonSRX climb2 = {8};
TalonSRX m_intake = {9};
TalonSRX m_slide = {10};
rev::CANSparkMax m_lift{61, rev::CANSparkMax::MotorType::kBrushless};
rev::CANEncoder liftEncoder{m_lift};
frc::AnalogInput ballSensor{0};
int ballCount=0;
bool collectingBalls=false;
bool beginAutonomous=true;
bool loadingBall=false;
frc::Timer SBtimer;
bool shootBallsLong=false;
frc::Servo *camServo = new frc::Servo(0);
double servoValue=0;  
TalonSRX srx = {13};
  
  

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