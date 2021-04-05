/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <fstream>
#include "frc/PWMTalonFX.h"
#include "ctre/Phoenix.h"
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/XboxController.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/Timer.h>
#include <cmath>
#include <frc/util/color.h>
#include "rev/CANSparkMax.h"
#include "frc/WPILib.h"
#include "frc/ADXRS450_Gyro.h"
#include "cameraserver/CameraServer.h"
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "Robot.h"
#include "kinematics.h"
#include "Autonomous.h"
#include "Slolum.h"

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////CLASS DEFINITION /////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
class Robot : public frc::TimedRobot {

  ofstream autoLog;

  //////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////CONSTRUCTOR/////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////

  public: Robot() { //runs on robot boot
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

  void TeleopInit() { //Runs when Teleop Starts
    timer.Reset();
    timer.Start();    
    autoLog.open ("/U/log.txt");
    autoLog << "autoDriveSum,autoTurn,autoTimer\n";
    wpi::outs() << "autoDriveSum,autoTurn,autoTimer\n";
  }

  void TeleopPeriodic() { //Loops During Teleop

    //Sends Motor Values to SmartDashboard to be logged as CSV
    frc::SmartDashboard::PutNumber("Speed", driveSum);
    frc::SmartDashboard::PutNumber("turn", turn);
    frc::SmartDashboard::PutNumber("Timer", timer.Get());
    //frc::SmartDashboard::PutNumber("Accelerometer", accelerometer.GetY());

    //Drivetrain Control
    //wpi::outs() << "Turn: " << (teleopTurnAvgSum / teleopAvgLength) << "\n";
    turn = (driveSpeed * joystickLinearScaledDeadband(driver.GetX(frc::GenericHID::JoystickHand::kRightHand))) + speed;
    driveSum = (-driveSpeed * joystickLinearScaledDeadband(driver.GetY(frc::GenericHID::JoystickHand::kLeftHand)));
    //m_robotDrive->ArcadeDrive(driveSum, turn);
    m_robotDrive->ArcadeDrive(driveSum, turn);
    //wpi::outs() << driveSum << "," << turn << "," << timer.Get() << "\n";
    autoLog << driveSum << "," << turn << "," << timer.Get() << "\n";
    //Update Gamedata
    gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
    //Slow drivespeed
    if (driver.GetTriggerAxis(frc::GenericHID::JoystickHand::kRightHand) > 0.2) {
      driveSpeed = 0.55;
    }
    else {
      driveSpeed = speedFast;
    }
    //drivespeed X Button toggle
    if (driver.GetXButtonPressed() && (driveSpeed == speedFast)) {
      driveSpeed = speedSlow;
    }
    else if (driver.GetXButtonPressed() && (driveSpeed == speedSlow)) {
      driveSpeed = speedFast;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////COPILOT FUNCTIONS//////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////

    //--------------------------ShooterPID------------------------------------------------
    double shooter_SetPoint = 0.0;// = MaxRPM*m_stick.GetY()
    //ShortRange
    if (copilot.GetBumper(frc::GenericHID::JoystickHand::kLeftHand)) {
      shooter_SetPoint = 3510;
    }
    //MediumRange
    else if (copilot.GetBumper(frc::GenericHID::JoystickHand::kRightHand)) {
      shooter_SetPoint = 3655;
    }
    //LongRange
    else if (copilot.GetTriggerAxis(frc::GenericHID::JoystickHand::kRightHand) > .2) {
      shooter_SetPoint = 4040;
    }
    else {
      shooter_SetPoint = 0;
    }
    //Update Shooter
    m_shooterPidController.SetReference(shooter_SetPoint, rev::ControlType::kVelocity);
    //------------------------------------------------------------------------------------

    //collect balls
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
      //std::cout << "RESETTING TIMER \n";
      //std::cout << "sensor voltage is " << ballSensor.GetVoltage() << "\n";
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
      if (driveSpeed == speedFast) {
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
      movnum= 0;
      movtmr.Reset();
      gyro.Reset();
      bearing = 0;      
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
    ////////////////////////////A BUTTON PRESSED///////////////////////////////////////////////////
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
    /////////////////////Left Trigger ////////////////////////
    if (copilot.GetTriggerAxis(frc::GenericHID::JoystickHand::kLeftHand) > 0.2) {

    }
    /////////////////////Right Trigger ///////////////////////
    if (copilot.GetTriggerAxis(frc::GenericHID::JoystickHand::kRightHand) > 0.2) {
      
    }
    /////////////////////LOADING BALL/////////////////////////
    if (loadingBall){
      //std::cout << "IN LOAD BALL bool\n";
      collectingBalls=false;

      if (LBtimer.Get() < 0.5){
        m_feeder.Set(ControlMode::PercentOutput, -0.5);
          //std::cout << "IN LOAD BALL bool1111 " << LBtimer.Get() << "\n";
      }
      else {
        m_feeder.Set(ControlMode::PercentOutput, 0.0);
        //std::cout << "IN LOAD BALL bool2222\n";
      }
      if ((LBtimer.Get() > 0.5)&&(liftEncoder.GetPosition() > -11)){
        //std::cout << "IN LOAD BALL bool3333\n";
        m_lift.Set(-0.5);      
      }
      if ((LBtimer.Get() > 0.5)&&(liftEncoder.GetPosition() < -11)){
        //std::cout << "IN LOAD BALL bool4444\n";
        m_lift.Set(0.0);
        liftEncoder.SetPosition(0);
        loadingBall=false;
      }
    }    
  }

  //////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////AutonomousPeriodic///////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////

  void AutonomousInit() { // Runs Once when Autonomous Starts
    timer.Reset();
    timer.Start();
    i = 0;
  }

  void AutonomousPeriodic(){ //Loops During Autonomous
    //m_robotDrive->ArcadeDrive(autoDriveSum[i], autoTurn[i]);
    if (i < (sizeof(autoDriveSum) / sizeof(autoDriveSum[0]))) {
      wpi::outs() << autoDriveSum[i] << "," << autoTurn[i] << "," << autoTimer[i] << "," << timer.Get() << "\n";
      m_robotDrive->ArcadeDrive(autoDriveSum[i],autoTurn[i]);      
      i++;
    }
    else {
      m_robotDrive->ArcadeDrive(0.0,0.0);
      wpi::outs() << "0,0,0\n";
    }
  }

};

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif