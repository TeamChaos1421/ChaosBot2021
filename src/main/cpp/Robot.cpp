/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include "frc/PWMTalonFX.h"
#include "ctre/Phoenix.h"
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/XboxController.h>
#include <frc/SpeedControllerGroup.h>
#include <iostream>
#include <frc/Timer.h>
#include <cmath>
#include <frc/SpeedControllerGroup.h>
#include <frc/util/color.h>
#include <frc/Timer.h>
#include "rev/CANSparkMax.h"
#include "frc/WPILib.h"
#include "frc/ADXRS450_Gyro.h"
#include <frc/util/color.h>
#include "cameraserver/CameraServer.h"
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "Robot.h"
#include "kinematics.h"

//////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////CLASS DEFINITION /////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
class Robot : public frc::TimedRobot {
  
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

  double driveSpeed=speedFast;
  double turn;
  
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

  public:
  frc::Timer SBAtimer;
  bool shootBallsShort=false;
  int movnum = 0;
  frc::Timer movtmr;
  double lcounts, rcounts, mtime;
  frc::ADXRS450_Gyro gyro;
  double bearing =0;
  bool linearDone=false;
  bool angleDone=false;
  frc::Timer LBtimer;

  //////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////CONSTRUCTOR///////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////
  Robot() {

    //   m_leftMotor2a->Follow(*m_leftMotor1a);
    //   m_rightMotor2a->Follow(*m_rightMotor1a);


    //-----------------------ShooterPID setup-------------------------------------------------
    m_shooter.RestoreFactoryDefaults();
    m_shooterPidController.SetP(shooter_kP);
    m_shooterPidController.SetI(shooter_kI);
    m_shooterPidController.SetD(shooter_kD);
    m_shooterPidController.SetIZone(shooter_kIz);
    m_shooterPidController.SetFF(shooter_kFF);
    m_shooterPidController.SetOutputRange(shooter_kMinOutput, shooter_kMaxOutput);
    //----------------------------------------------------------------------------------------

    m_leftMotor1a->ConfigFactoryDefault();
    m_rightMotor1a->ConfigFactoryDefault();
    m_leftMotor2a->ConfigFactoryDefault();
    m_rightMotor1a->ConfigFactoryDefault();
    m_leftMotor1a->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);    
    m_leftMotor1a->ConfigNominalOutputForward(0,50);
    m_leftMotor1a->ConfigNominalOutputReverse(0,50);
    m_leftMotor1a->ConfigPeakOutputForward(maxSpeed,50);
    m_leftMotor1a->ConfigPeakOutputReverse(-maxSpeed,50);
    m_leftMotor2a->ConfigNominalOutputForward(0,50);
    m_leftMotor2a->ConfigNominalOutputReverse(0,50);
    m_leftMotor2a->ConfigPeakOutputForward(maxSpeed,50);
    m_leftMotor2a->ConfigPeakOutputReverse(-maxSpeed,50);
    m_leftMotor1a->Config_kF(lkPIDLoopIdx, 0.0, 50);
    m_leftMotor1a->Config_kP(lkPIDLoopIdx, 0.1, 50);
    m_leftMotor1a->Config_kI(lkPIDLoopIdx, 0.0, 50);
    m_leftMotor1a->Config_kD(lkPIDLoopIdx, 0.0, 50);
    m_rightMotor1a->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
    m_rightMotor1a->ConfigNominalOutputForward(0,50);
    m_rightMotor1a->ConfigNominalOutputReverse(0,50);
    m_rightMotor1a->ConfigPeakOutputForward(maxSpeed,50);
    m_rightMotor1a->ConfigPeakOutputReverse(-maxSpeed,50);
    m_rightMotor2a->ConfigNominalOutputForward(0,50);
    m_rightMotor2a->ConfigNominalOutputReverse(0,50);
    m_rightMotor2a->ConfigPeakOutputForward(maxSpeed,50);
    m_rightMotor2a->ConfigPeakOutputReverse(-maxSpeed,50);
    m_rightMotor1a->Config_kF(rkPIDLoopIdx, 0.0, 50);
    m_rightMotor1a->Config_kP(rkPIDLoopIdx, 0.1, 50);
    m_rightMotor1a->Config_kI(rkPIDLoopIdx, 0.0, 50);
    m_rightMotor1a->Config_kD(rkPIDLoopIdx, 0.0, 50);    
    gyro.Reset();
    movtmr.Reset();
    std::string gameData;
    SBAtimer.Reset();

    //ballSensor.SetErrorRange()
    frc::CameraServer::GetInstance()->StartAutomaticCapture();
    cs::CvSink cvSink = frc::CameraServer::GetInstance()->GetVideo();
    cs::CvSource outputStream = frc::CameraServer::GetInstance()->PutVideo("Blur",640,480);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////TeleopPeriodic//////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////

  void TeleopPeriodic() { 
    //Runs Once:
    if (init) {
      frc::SmartDashboard::PutNumber("Accelerometer", smartDashboardTest);
      init = false;
    }
    //Prints Accelerometer to SmartDashboard
    frc::SmartDashboard::PutNumber("Accelerometer", accelerometer.GetY());

    //Drivetrain Control
    turn = (driveSpeed * joystickLinearScaledDeadband(driver.GetX(frc::GenericHID::JoystickHand::kRightHand))) + speed;
    m_robotDrive->ArcadeDrive(-driveSpeed * joystickLinearScaledDeadband(driver.GetY(frc::GenericHID::JoystickHand::kLeftHand)), turn);
    gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
    if (driver.GetBumperPressed(frc::GenericHID::JoystickHand::kRightHand)) {
      driveSpeed = 1.0;
    }
    if (driver.GetBumperReleased(frc::GenericHID::JoystickHand::kRightHand)) {
      driveSpeed = speedFast;
    }

    /////////////////////////////////COPILOT FUNCTIONS//////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////

    //--------------------------ShooterPID------------------------------------------------
    double shooter_SetPoint = 0.0;// = MaxRPM*m_stick.GetY()
    if (copilot.GetBumper(frc::GenericHID::JoystickHand::kLeftHand) && true) {
      shooter_SetPoint = 3500;
    }
    else if (copilot.GetBumper(frc::GenericHID::JoystickHand::kRightHand)) {
      shooter_SetPoint = 3655;
    }
    else if (copilot.GetTriggerAxis(frc::GenericHID::JoystickHand::kRightHand) > .2) {
      shooter_SetPoint = 4040;
    }
    else {
      shooter_SetPoint = 0;
    }
    m_shooterPidController.SetReference(shooter_SetPoint, rev::ControlType::kVelocity);
    //------------------------------------------------------------------------------------

    ///collect balls
    if (copilot.GetAButton()) {
      m_intake.Set(ControlMode::PercentOutput, 0.4);
      collectingBalls=true;
    }
    else{
      m_intake.Set(ControlMode::PercentOutput, 0);
      collectingBalls=false;
    }
    if (collectingBalls && (ballSensor.GetVoltage()<1)){
        liftEncoder.SetPosition(0);
        std::cout << "RESETTING TIMER \n";
        std::cout << "sensor voltage is " << ballSensor.GetVoltage() << "\n";
        LBtimer.Reset();
        LBtimer.Start();
        loadingBall=true;
      }
    //repel balls
    if (copilot.GetBButton()) {
      m_intake.Set(ControlMode::PercentOutput, -0.5);
    }
    //advance lift
    if (copilot.GetXButton()) {
      m_lift.Set(-0.5);
    }
    else if (copilot.GetXButtonReleased()) {
    m_lift.Set(0.0);
    }
    //reverse lift
    if (copilot.GetYButton()) {
    m_lift.Set(0.5);
    }
    else if (copilot.GetYButtonReleased()) {
    m_lift.Set(0.0);
    }
    //Climber Controls
    climb1.Set(ControlMode::PercentOutput, copilot.GetY(frc::GenericHID::JoystickHand::kLeftHand));
    climb2.Set(ControlMode::PercentOutput, copilot.GetY(frc::GenericHID::JoystickHand::kLeftHand));
    m_slide.Set(ControlMode::PercentOutput, copilot.GetX(frc::GenericHID::JoystickHand::kRightHand));
    ////////////////////////////X BUTTON PRESSED///////////////////////////////////////////////////
    if (driver.GetXButtonPressed()) {
      if (driveSpeed == 0.7) {
        driveSpeed = speedSlow;
      }
      else {
        driveSpeed = speedFast;
      }
    }
    ////////////////////////////Y BUTTON PRESSED///////////////////////////////////////////////////
    if (driver.GetYButtonPressed()) {
      
      m_leftMotor1a->SetSelectedSensorPosition(0);
      m_rightMotor1a->SetSelectedSensorPosition(0);
      autoFlag = false;
      lcounts = 0;
      rcounts = 0;
      mtime = 0;
      movnum=0;
      movtmr.Reset();
      gyro.Reset();
      //turnFlag=false;
      bearing =0;      
    }
        ////////////////////////////B BUTTON PRESSED///////////////////////////////////////////////////
    if (driver.GetBButtonPressed()) {

     if (camServo->GetPosition() == 0.3){
       camServo->Set(0.5);
     }

     else if (camServo->GetPosition() == 0.5){
       camServo->Set(0.75);
     }
     else{
       camServo->Set(0.3);
     }
     
     
   }
    if (driver.GetAButtonPressed()){
          if (camServo->GetPosition() == 0.75){
       camServo->Set(0.5);
     }

     else if (camServo->GetPosition() == 0.5){
       camServo->Set(0.3);
     }
     else{
       camServo->Set(0.75);
     }
   }
    //////////////////LOADING BALL////////////////////////////
    if (loadingBall){
      std::cout << "IN LOAD BALL bool\n";
      collectingBalls=false;

      if (LBtimer.Get() < 0.5){
        m_feeder.Set(ControlMode::PercentOutput, -0.5);
              std::cout << "IN LOAD BALL bool1111 " << LBtimer.Get() << "\n";
      }
      else {
        m_feeder.Set(ControlMode::PercentOutput, 0.0);
              std::cout << "IN LOAD BALL bool2222\n";
      }

      if ((LBtimer.Get() > 0.5)&&(liftEncoder.GetPosition() > -11)){
      std::cout << "IN LOAD BALL bool3333\n";
        m_lift.Set(-0.5);      
      }
      if ((LBtimer.Get() > 0.5)&&(liftEncoder.GetPosition() < -11)){
              std::cout << "IN LOAD BALL bool4444\n";
        m_lift.Set(0.0);
        liftEncoder.SetPosition(0);
        loadingBall=false;
        //LBtimer.Reset();
      }
    }
    
  }

  //////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////AutonomousPeriodic///////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////

  double avg[avgLength];
  double a[11] = {0,0,0,0,0,0,0,0,0,0,0};
  double duration = 2;
  double sum;
  
  void AutonomousPeriodic(){
  
    if (beginAutonomous){
      timer.Reset();
      timer.Start();
      frc::SmartDashboard::GetNumber("duration: ", duration);
      /**if (timer.Get() < duration) {
        m_robotDrive->ArcadeDrive(0.5, 0.0);
      }
      else {
        m_robotDrive->ArcadeDrive(0.0, 0.0);
      }**/

      for (int value = (avgLength - 1); value <= 0; value--) {
          avg[(value + 1)] = avg[value];
      }
      avg[0] = (accelerometer.GetY() / 9.8);

      for (int value = 0; value <= (avgLength - 1); value++) {
        sum = sum + avg[value];
      }
      
    
      a[0] = sum / avgLength;
      a[9] = timer.Get();

      for (int i = 0; i <= 10; i++) {
        wpi::outs() << a[i] << ", ";
      }
      wpi::outs() << "\n" << "\n";

      kinematics(a);
      
      frc::SmartDashboard::PutNumber("Y-Pos", a[7]);
      frc::SmartDashboard::PutNumber("Test", accelerometer.GetY());
    }
  }

};

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif