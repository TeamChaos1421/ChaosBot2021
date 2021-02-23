/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc/PWMTalonFX.h"
#include "ctre/Phoenix.h"
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include "rev/CANSparkMax.h"
#include <frc/XboxController.h>
#include <frc/SpeedControllerGroup.h>
#include <iostream>
#include <frc/Timer.h>
#include <cmath>
#include <frc/SpeedControllerGroup.h>
#include "frc/WPILib.h"
#include "frc/ADXRS450_Gyro.h"
#include <frc/util/color.h>
#include "cameraserver/CameraServer.h"
#include <frc/util/color.h>
#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"

#include "C:\Users\Drew Helgerson\Desktop\Code Saves\Color Sensor files\ControlPanel.h"
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

//setting slow and fast speeds
double speedFast = .7;
double speedSlow = .45;

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


//////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////CLASS DEFINITION /////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
class Robot : public frc::TimedRobot {

  WPI_TalonFX *m_leftMotor1a = new WPI_TalonFX(23);
  WPI_TalonFX *m_leftMotor2a = new WPI_TalonFX(22);
  WPI_TalonFX *m_rightMotor1a = new WPI_TalonFX(21);
  WPI_TalonFX *m_rightMotor2a = new WPI_TalonFX(20);
  frc::SpeedControllerGroup *m_left = new frc::SpeedControllerGroup(*m_leftMotor1a, *m_leftMotor2a);
  frc::SpeedControllerGroup *m_right = new frc::SpeedControllerGroup(*m_rightMotor1a, *m_rightMotor2a);
//  frc::DifferentialDrive *m_robotDrive = new frc::DifferentialDrive(*m_leftMotor1a, *m_rightMotor1a);
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
 
  //element 1 is 0 or 1
  //0 is straight
  //1 is turn
  //2 is check turn
  //3 is unload
  double turningSpeed=0.35;
  int maxmoves=3;
  double moves[20][3] = {
  {3, 0, 0}, 
  {0, -3.0, 3},
  {1, 90,turningSpeed}, 
  {2,0,0}, 
  {0, 3,2}, 
  {1, -90, turningSpeed},
  {2,0,0},
  {0, 3,2},
  {1, 180, turningSpeed},
  {2,0,0},
  {0, 3, 2}, 
  {1, 90, turningSpeed},
  {2,0,0}, 
  {0, 2.66, 2}, 
  {1, 90, turningSpeed},
  {2,0,0},
  {0, -2.25,2},
  {1, 180, turningSpeed},
  {2,0,0}
   };




  double cntpft = 14500;

  double maxSpeed=1;
  float angle;
	double offset;
	float speed;
  float absSpeed;
  ControlPanel CP;
  ControlPanel CM;
  bool PositionControlDone;
  bool RotationControlDone;
  //char turns;
  std::string gameData;


 // float targetAngle;

  TalonSRX m_feeder = {2};
  TalonSRX climb1 = {7};
  TalonSRX climb2 = {8};
  TalonSRX m_intake = {9};
  TalonSRX m_slide = {10};
 // TalonSRX srx = {13};
  rev::CANSparkMax m_shooter{60, rev::CANSparkMax::MotorType::kBrushless};
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

static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
  rev::ColorSensorV3 m_colorSensor{i2cPort};
  frc::Color detectedColor;
  frc::Color matchedColor;
  static constexpr frc::Color yellowStrip = frc::Color(0.321, 0.552, 0.126);
  static constexpr frc::Color redStrip = frc::Color(0.541, 0.330, 0.128);
  static constexpr frc::Color greenStrip = frc::Color(0.1939, 0.5531, 0.2530);
  static constexpr frc::Color blueStrip = frc::Color(0.1458, 0.4315, 0.4224);
  static constexpr frc::Color yellowWheel = frc::Color(0.320, 0.561, 0.119);
  static constexpr frc::Color redWheel = frc::Color(0.419, 0.334, 0.246);
  static constexpr frc::Color greenWheel = frc::Color(0.254, 0.579, 0.168);
  static constexpr frc::Color blueWheel = frc::Color(0.173, 0.447, 0.380);
  static constexpr frc::Color whiteStrip = frc::Color(0, 0, 0);
  static constexpr frc::Color falseYellowStrip = frc::Color(0.299, 0.478, 0.223);
  static constexpr frc::Color white = frc::Color(0.25647, 0.467896, 0.275757);

  rev::ColorMatch m_colorMatcher;
  std::string colorString;
  std::string lastColorString;
  double confidence =0.0;
  bool countingColors=false;
  int colorCount=0;
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

//limelight variables
  float targetX;
  float targetY;
  float targetA;
  bool targetSeek=false;
  bool finetargetSeek=false;
  std::shared_ptr<NetworkTable> table;
  double targetSpeed=0.4;
  double finetargetSpeed=0.35;
  int targetfoundCounter=0;
  double currentAngle;
  double startAngle;

//////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////CONSTRUCTOR///////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
  Robot() {

 //   m_leftMotor2a->Follow(*m_leftMotor1a);
 //   m_rightMotor2a->Follow(*m_rightMotor1a);

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

//    ballSensor.SetErrorRange()
  frc::CameraServer::GetInstance()->StartAutomaticCapture();
  cs::CvSink cvSink = frc::CameraServer::GetInstance()->GetVideo();
  cs::CvSource outputStream = frc::CameraServer::GetInstance()->PutVideo("Blur",640,480);
  

    m_colorMatcher.AddColorMatch(yellowStrip);
    m_colorMatcher.AddColorMatch(redStrip);
    m_colorMatcher.AddColorMatch(greenStrip);
    m_colorMatcher.AddColorMatch(blueStrip);
    m_colorMatcher.AddColorMatch(white);
  
  
  table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  
  }




//////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////TELEOP PERIODIC///////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
  void TeleopPeriodic() { 

    turn = (driveSpeed * joystickLinearScaledDeadband(driver.GetX(frc::GenericHID::JoystickHand::kRightHand))) + speed;

    m_robotDrive->ArcadeDrive(-driveSpeed * joystickLinearScaledDeadband(driver.GetY(frc::GenericHID::JoystickHand::kLeftHand)), turn);
    gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
    if (driver.GetBumperPressed(frc::GenericHID::JoystickHand::kRightHand)) {
      driveSpeed = 1.0;
    }
    if (driver.GetBumperReleased(frc::GenericHID::JoystickHand::kRightHand)) {
      driveSpeed = speedFast;
    }



//std::cout << "GAMEDATA COLOR IS " << gameData << "\n";




    if (countingColors){
      lastColorString=colorString;
      detectedColor = m_colorSensor.GetColor();
      matchedColor = m_colorMatcher.MatchClosestColor(detectedColor,confidence);
      
      if (matchedColor == yellowStrip){
        colorString = "yellow";
      }
      else if (matchedColor == redStrip){
        colorString = "red";
      }
      else if (matchedColor == greenStrip){
        colorString = "green";
      }
      else if (matchedColor == blueStrip){
        colorString = "blue";
      }
      else if (matchedColor == white){
        colorString = colorString;
      }
      
     /*
      if (matchedColor == yellowWheel){
        colorString = "yellow";
      }
      else if (matchedColor == redWheel){
        colorString = "red";
      }
      else if (matchedColor == greenWheel){
        colorString = "green";
      }
      else if (matchedColor == blueWheel){
        colorString = "blue";
      }*/
      
      if (lastColorString.compare(colorString)!=0){
        std::cout << "COLOR CHANGED. Color is " << colorString << "\n";
        std::cout << "NUM CHANGES IS " << colorCount << "\n";
//        std::cout << "detected Color " << detectedColor.red << " " << detectedColor.green << " " << detectedColor.blue << "\n";
        colorCount++;
 //       std::cout << "TURNING THE MOTOR. ColorCount is " << colorCount << "\n";
      }
      if (colorCount >= 32){
        std::cout << "FOUND 32 COLORS!!  We're DONE!\n";
        colorCount=0;
        countingColors=false;
         srx.Set(ControlMode::PercentOutput, 0.0);
       }
    }
if (copilot.GetBumper(frc::GenericHID::JoystickHand::kLeftHand) && true) {
    m_shooter.Set(0.72);
    // srx.Set(ControlMode::PercentOutput, 0.15);
}
if (copilot.GetBumperReleased(frc::GenericHID::JoystickHand::kLeftHand) && true) {
    m_shooter.Set(0.0);
    // srx.Set(ControlMode::PercentOutput, 0.0);
}

if (copilot.GetYButton() && true) {
    m_lift.Set(0.5);
    // srx.Set(ControlMode::PercentOutput, 0.15);
}
if (copilot.GetYButtonReleased() && true) {
    m_lift.Set(0.0);
    // srx.Set(ControlMode::PercentOutput, 0.0);
}

if (copilot.GetXButton() && true) {
    m_lift.Set(-0.75);
    // srx.Set(ControlMode::PercentOutput, 0.15);
}
if (copilot.GetXButtonReleased() && true) {
    m_lift.Set(0.0);
    // srx.Set(ControlMode::PercentOutput, 0.0);
}


if (copilot.GetBumper(frc::GenericHID::JoystickHand::kRightHand) && true) {
      detectedColor = m_colorSensor.GetColor();
      std::cout << "detected Color " << detectedColor.red << " " << detectedColor.green << " " << detectedColor.blue << "\n";
}
if (copilot.GetBumper(frc::GenericHID::JoystickHand::kLeftHand)&& true) {
      detectedColor = m_colorSensor.GetColor();
   //   std::cout << "detected Color " << detectedColor.red << " " << detectedColor.green << " " << detectedColor.blue << "\n";
      matchedColor = m_colorMatcher.MatchClosestColor(detectedColor,confidence);
      
      if (matchedColor == yellowStrip){
        colorString = "yellow";
      }
      else if (matchedColor == redStrip){
        colorString = "red";
      }
      else if (matchedColor == greenStrip){
        colorString = "green";
      }
      else if (matchedColor == blueStrip){
        colorString = "blue";
      }
      srx.Set(ControlMode::PercentOutput, 0.35);
      
     /*
      if (matchedColor == yellowWheel){
        colorString = "yellow";
      }
      else if (matchedColor == redWheel){
        colorString = "red";
      }
      else if (matchedColor == greenWheel){
        colorString = "green";
      }
      else if (matchedColor == blueWheel){
        colorString = "blue";
      }*/
    //  std::cout << "COLOR IS " << colorString << "\n";
      countingColors=true;
      //colorCount=0;


    }









///COPILOT FUNCTIONS//////////////////////////////////////////

///rotation
    if (copilot.GetBumperPressed(frc::GenericHID::JoystickHand::kLeftHand)&&false) {
      CP.DetectColor();
      if(!RotationControlDone){
        RotationControlDone=CP.RotationControl();
      }   
    }
///position
    if (copilot.GetBumperPressed(frc::GenericHID::JoystickHand::kRightHand)&&false) {
      CP.DetectColor();
      if(!PositionControlDone){
        PositionControlDone=CP.PositionControl(gameData[0]);
      }      
    }
///reset control panel
    if (copilot.GetBumperReleased(frc::GenericHID::JoystickHand::kLeftHand)&&false) {
      CP.ResetValues();
      }      
    if (copilot.GetBumperReleased(frc::GenericHID::JoystickHand::kRightHand)&&false) {
      CP.ResetValues();
      }      

///collect balls
  if (copilot.GetAButton()) {
    m_intake.Set(ControlMode::PercentOutput, 0.5);
    collectingBalls=true;
  }
  else{
    m_intake.Set(ControlMode::PercentOutput, 0);
    collectingBalls=false;
  }
  if (collectingBalls && (ballSensor.GetVoltage()<1)){
      liftEncoder.SetPosition(0);
//      m_intake.Set(ControlMode::PercentOutput, 0);
      //loadBall();
      std::cout << "RESETTING TIMER \n";
      std::cout << "sensor voltage is " << ballSensor.GetVoltage() << "\n";
      LBtimer.Reset();
  	  LBtimer.Start();
      loadingBall=true;
    }


///repel balls
  if (copilot.GetBButton()) {
    m_intake.Set(ControlMode::PercentOutput, -0.5);
  }
  


if ((copilot.GetXButton() & false)) {
    //shootBalls();
    SBtimer.Reset();
  	SBtimer.Start();
    shootBallsShort=true;
}

if ((copilot.GetYButtonPressed() & false)) {
  //shootBallsClose();
    SBtimer.Reset();
  	SBtimer.Start();
    shootBallsLong=true;
      //srx.Set(ControlMode::PercentOutput,-0.5);
}
 //if (copilot.GetYButtonReleased()) {
   // srx.Set(ControlMode::PercentOutput,0);
  //}

 climb1.Set(ControlMode::PercentOutput, copilot.GetY(frc::GenericHID::JoystickHand::kLeftHand));
 climb2.Set(ControlMode::PercentOutput, copilot.GetY(frc::GenericHID::JoystickHand::kLeftHand));
 m_slide.Set(ControlMode::PercentOutput, copilot.GetX(frc::GenericHID::JoystickHand::kRightHand));
 

////////////////////////////X BUTTON PRESSED///////////////////////////////////////////////////
    if (driver.GetXButtonPressed()) {
std::cout << " DRIVER X BUTTON\n";

      if (driveSpeed == 0.7) {
        driveSpeed = speedSlow;
      }
      else {
        driveSpeed = speedFast;
      }
      /*
      std::cout << "Left motor " << (m_leftMotor1a->GetSelectedSensorPosition()) << "\n";
      std::cout << "lcounts is " << lcounts << "\n";
      std::cout << "Right motor " << (m_rightMotor1a->GetSelectedSensorPosition()) << "\n";
      std::cout << "rcounts is " << rcounts << "\n";
      std::cout << "Gyro Angle is " << gyro.GetAngle() << "\n";
      */
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
          if (camServo->GetPosition() == 0.7){
       camServo->Set(0.5);
     }

     else if (camServo->GetPosition() == 0.5){
       camServo->Set(0.3);
     }
     else{
       camServo->Set(0.7);
     }
   }


  if (driver.GetBumperPressed(frc::GenericHID::JoystickHand::kRightHand)) {
    targetSeek = false;
    finetargetSeek = false;
  }
  if ((driver.GetBumperPressed(frc::GenericHID::JoystickHand::kLeftHand)) && false) {
    // Get limelight table for reading tracking data
    wpi::outs() << "LEFT BUMPER PRESSED\n";
  //  std::shared_ptr<NetworkTable> table = NetworkTable::GetTable("limelight");
    //std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    targetX = table->GetNumber("tx", 0);
    targetY = table->GetNumber("ty", 0);
    targetA = table->GetNumber("ta", 0);
    wpi::outs() << "target X is  " << targetX << "\n";
    wpi::outs() << "target Y is  " << targetY << "\n";
    wpi::outs() << "target A is  " << targetA << "\n";
    startAngle = gyro.GetAngle();
    targetSeek=true;
  }

/////////////////SEEKING TARGET///////////////////////////
  if (targetSeek){
  //  std::cout << "IN TARGET SEEK\n";
    targetX = table->GetNumber("tx", 0);
    targetY = table->GetNumber("ty", 0);
    targetA = table->GetNumber("ta", 0);
    angle = targetX;
    currentAngle = gyro.GetAngle();
    if ((currentAngle >= (startAngle+360)) || (currentAngle <= (startAngle-360))){
      speed=0;
      targetSeek=false;
    }
    else if ((angle > 0.5)||(targetA < 3.0))
		  speed = targetSpeed;
		else if (angle < -0.5)
		  speed = -targetSpeed;
    else if ((angle > -0.5) && (angle < 0.5)){
      speed = 0;
      targetSeek=false;
      finetargetSeek=true;
    }
std::cout << "IN TARGET SEEK X IS " << speed << " " << angle << "\n";
    }

  if (finetargetSeek){
  //  std::cout << "IN TARGET SEEK\n";
    targetX = table->GetNumber("tx", 0);
    targetY = table->GetNumber("ty", 0);
    targetA = table->GetNumber("ta", 0);
    std::cout << targetX;
    std::cout << targetY;
    angle = targetX;
    if(angle > 0.35)
		  speed = finetargetSpeed;
		if (angle < -0.35)
		  speed = -finetargetSpeed;
    if ((angle > -0.35) && (angle < 0.35)){
      speed = 0;
      targetfoundCounter++;
      if (targetfoundCounter >= 5){
        finetargetSeek=false;
        targetfoundCounter=0;
      }
    }
    std::cout << "IN FINE TARGET SEEK X IS " << speed << " " << angle << "\n";
    }


//////////////////LOADING BALL////////////////////////////
    if (loadingBall){
      std::cout << "IN LOAD BALL bool\n";
      collectingBalls=false;
//      frc::Timer LBtimer;

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
    
  ///////////////////////////////////////////////////////////

  /////////////////SHOOTBALLSLONG/////////////
    if (shootBallsLong){  
      std::cout << "IN SHOOT BALLS LONG\n";

      if (SBtimer.Get() < 0.25){
        m_lift.Set(0.5);
      }
      if ((SBtimer.Get() < 2.0)&&(SBtimer.Get() > 0.25)){
        m_shooter.Set(0.6);
        m_lift.Set(0.0);
      }
      if ((SBtimer.Get() <10)&&(SBtimer.Get() > 2.0)) { 
        m_lift.Set(-0.2);
        m_feeder.Set(ControlMode::PercentOutput, -0.4);
      }
      if (SBtimer.Get() > 10.0){
        m_shooter.Set(0);
        m_lift.Set(0);
        m_feeder.Set(ControlMode::PercentOutput, 0);
        shootBallsLong=false;
      }
    }
    if (shootBallsShort){  
      std::cout << "IN SHOOT BALLS SHORT \n";

      if (SBtimer.Get() < 0.250){
        m_lift.Set(0.5);
      }
      if ((SBtimer.Get() < 2.0)&&(SBtimer.Get() > 0.25)){
        m_shooter.Set(0.7);
        m_lift.Set(0.0);
      }
      if ((SBtimer.Get() <6)&&(SBtimer.Get() > 2.0)) { 
        m_lift.Set(-0.4);
        m_feeder.Set(ControlMode::PercentOutput, -0.3);      
      }
      if (SBtimer.Get() > 7){
        m_shooter.Set(0);
        m_lift.Set(0);
        m_feeder.Set(ControlMode::PercentOutput, 0);
        shootBallsShort=false;
      }
    }

  //////////////////////////////////////////////
  
  /////////////////////
  ////////////

  }


////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
//AUTONOMOUS

void AutonomousPeriodic(){

//SBAtimer.Reset();
if (beginAutonomous){
  std::cout << "IN AUTONOMOUS PERIODICAAAAA \n";
 // srx.Set(ControlMode::PercentOutput,-0.5);
//  m_leftMotor1a->SetSelectedSensorPosition(0);
//      m_rightMotor1a->SetSelectedSensorPosition(0);

      autoFlag = true;
      lcounts = 0;
      //rcounts = 0;
      mtime = 0;
//      movnum=0;
//      movtmr.Reset();
//      gyro.Reset();
      bearing=0;


      m_leftMotor2a->SetNeutralMode(NeutralMode::Coast);
      m_rightMotor2a->SetNeutralMode(NeutralMode::Coast);



 
////////////////////////////STRAIGHT LINE MOVE//////////////////////////////////////////////
      if (moves[movnum] [0]==0.00) {
        std::cout << "STRAIGHT MOVE\n";       
        lcounts = moves[movnum] [1] *cntpft;
        //rcounts = moves[movnum] [1] *cntpft;
        mtime = moves[movnum] [2];
        movtmr.Start();
      
        if (movtmr.Get() <= mtime) {
          m_leftMotor1a->Set(ControlMode::Position, lcounts*movtmr.Get()/mtime);
          m_rightMotor1a->Set(ControlMode::Position, -lcounts*movtmr.Get()/mtime);
        }
        else {
          m_leftMotor1a->Set(ControlMode::Position, lcounts);
          m_rightMotor1a->Set(ControlMode::Position, -lcounts);
        } 

        if (((std::abs((double)(m_leftMotor1a->GetSelectedSensorPosition() - (lcounts)))) < 1000) &&
        ((std::abs((double)(m_rightMotor1a->GetSelectedSensorPosition() + (lcounts)))) < 1000)) {
          movnum++;
          std::cout << "finished straight move\n";
          movtmr.Reset();
          m_leftMotor1a->SetSelectedSensorPosition(0);
          m_rightMotor1a->SetSelectedSensorPosition(0);
          m_leftMotor1a->Set(ControlMode::Position, 0);
          m_rightMotor1a->Set(ControlMode::Position, 0);
          linearDone=true;
        }
      }
    



    ////////////////////////////TURN MOVE//////////////////////////////////////////////////////
      else if (moves[movnum] [0]==1.00) {
        std::cout << "TURNING MOVE\n";
        if (linearDone){
          movtmr.Reset();
          gyro.Reset();
          // turnFlag =true;
          bearing = moves[movnum] [1];
          m_leftMotor1a->SetSelectedSensorPosition(0);
          m_rightMotor1a->SetSelectedSensorPosition(0);
          absSpeed=moves[movnum][2];
          linearDone=false;
        }

    		angle = gyro.GetAngle();
		    offset = bearing - angle;
      
		    if(offset < 0)
			    speed = -absSpeed;
		    else
			    speed = absSpeed;

        m_robotDrive->ArcadeDrive(0, speed);

        if ( abs(offset) < 1 ) {
          angleDone=true;
          movnum++;
        }
      }


////////////////////////////////CHECK THE TURN///////////////////////////////////////
      else if (moves[movnum] [0]==2.00) {
        std::cout << "CHECK TURNING MOVE\n";
        std::cout << "Gyro Angle is " << gyro.GetAngle() << "\n";
        angle = gyro.GetAngle();
		    offset = bearing - angle;
        absSpeed=0.275;

		    if(offset < 0)
			    speed = -absSpeed;
		    else
			    speed = absSpeed;

        m_robotDrive->ArcadeDrive(0, speed);

        if ( fabs(offset) < 0.05 ) {
          std::cout << "offset is " << offset << "\n";
          m_leftMotor1a->SetSelectedSensorPosition(0);
          m_rightMotor1a->SetSelectedSensorPosition(0);
          movtmr.Reset();
          //gyro.Reset();
          bearing=0;
 //         angleDone=true;
          movnum++;
        }
      }
    else if (moves[movnum] [0]==3.00) {
              std::cout << "IN SHOOT BALLS AUTO 6666  set to false\n " << shootBallsShort <<"\n";
        std::cout << "movenum is " << movnum <<"\n";

      shootBallsShort=true;

              std::cout << "IN SHOOT BALLS AUTO 77777  set to false\n " << shootBallsShort <<"\n";

      //SBAtimer.Reset();
      SBAtimer.Start();
      //movnum++;

      
    }

////////////////////////////////ALL MOVES COMPLETE///////////////////////////////////////////      
      if (movnum == maxmoves) {
        std::cout << "ALL MOVES DONE \n";
        autoFlag = false;
        movnum = 0;
        m_leftMotor1a->SetSelectedSensorPosition(0);
        m_rightMotor1a->SetSelectedSensorPosition(0);
        movtmr.Reset();
        //gyro.Reset();
        bearing=0;

        m_leftMotor2a->SetNeutralMode(NeutralMode::Brake);
        m_rightMotor2a->SetNeutralMode(NeutralMode::Brake);
        m_leftMotor1a->SetNeutralMode(NeutralMode::Brake);
        m_rightMotor1a->SetNeutralMode(NeutralMode::Brake);
        beginAutonomous=false;
      }
    
    if (shootBallsShort){  
      std::cout << "IN SHOOT BALLS AUTO \n";

      //if (SBAtimer.Get() < 0.5){
      //  m_lift.Set(0.5);
     // }
            std::cout << "IN SHOOT BALLS AUTO " << SBAtimer.Get() <<"\n";
      if ((SBAtimer.Get() < 2.0)&&(SBAtimer.Get() > 0.5)){
        m_shooter.Set(0.60);
      }
      std::cout << "IN SHOOT BALLS AUTO 2222   " << SBAtimer.Get() <<"\n";
      if ((SBAtimer.Get() <6)&&(SBAtimer.Get() > 2.0)) { 
        m_lift.Set(-0.4);
        m_feeder.Set(ControlMode::PercentOutput, -0.5);      
      }

      std::cout << "IN SHOOT BALLS AUTO 3333   " << SBAtimer.Get() <<"\n";
        std::cout << "IN SHOOT BALLS AUTO 4444  set to false\n " << shootBallsShort <<"\n";
      if (SBAtimer.Get() > 6){
        shootBallsShort=false;
        std::cout << "IN SHOOT BALLS AUTO 55555  set to false\n " << shootBallsShort <<"\n";
        m_shooter.Set(0);
        m_lift.Set(0);
        m_feeder.Set(ControlMode::PercentOutput, 0);
        movtmr.Reset();
        m_leftMotor1a->SetSelectedSensorPosition(0);
        m_rightMotor1a->SetSelectedSensorPosition(0);
        movnum++;


      }
    }

}
}





};

#ifndef RUNNING_FRC_TESTS
int main()
{

  return frc::StartRobot<Robot>();
}
#endif
