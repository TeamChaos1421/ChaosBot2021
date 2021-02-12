#include "C:\Users\Drew Helgerson\Desktop\Code Saves\Color Sensor files\ControlPanel.h"
#include "ctre/Phoenix.h"
#include <frc/util/color.h>
#include "C:\Users\Drew Helgerson\Desktop\Code Saves\Color Sensor files\ColorSensorV3.h"
#include "C:\Users\Drew Helgerson\Desktop\Code Saves\Color Sensor files\ColorMatch.h"
#include <frc/smartdashboard/smartdashboard.h>

/**

   * Change the I2C port below to match the connection of your color sensor

   */

  static constexpr auto i2cPort = frc::I2C::Port::kOnboard;

/**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a 

   * parameter. The device will be automatically initialized with default 

   * parameters.

   */

  rev::ColorSensorV3 m_colorSensor{i2cPort};


  /**

   * A Rev Color Match object is used to register and detect known colors. This can 

   * be calibrated ahead of time or during operation.

   * 

   * This object uses a simple euclidian distance to estimate the closest match

   * with given confidence range.

   */

  /**

   * Note: Any example colors should be calibrated as the user needs, these

   * are here as a basic example.

   */

  static constexpr frc::Color kBlueTarget = frc::Color(0.143, 0.427, 0.429);

  static constexpr frc::Color kGreenTarget = frc::Color(0.197, 0.561, 0.240);

  static constexpr frc::Color kRedTarget = frc::Color(0.561, 0.232, 0.114);

  static constexpr frc::Color kYellowTarget = frc::Color(0.361, 0.524, 0.113);
  
  static int turns=0;

char ControlPanel::DetectColor(){
  rev::ColorMatch m_colorMatcher;

    m_colorMatcher.AddColorMatch(kBlueTarget);

    m_colorMatcher.AddColorMatch(kGreenTarget);

    m_colorMatcher.AddColorMatch(kRedTarget);

    m_colorMatcher.AddColorMatch(kYellowTarget);


    /**

     * The method GetColor() returns a normalized color value from the sensor and can be

     * useful if outputting the color to an RGB LED or similar. To

     * read the raw color, use GetRawColor().

     * 

     * The color sensor works best when within a few inches from an object in

     * well lit conditions (the built in LED is a big help here!). The farther

     * an object is the more light from the surroundings will bleed into the 

     * measurements and make it difficult to accurately determine its color.

     */

    frc::Color detectedColor = m_colorSensor.GetColor();
/**

     * Run the color match algorithm on our detected color

     */

    std::string colorString;

    double confidence = 0.0;

    frc::Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);


    if (matchedColor == kBlueTarget) {

      colorString = "Red";

    } else if (matchedColor == kRedTarget) {

      colorString = "Blue";
    } else if (matchedColor == kGreenTarget) {

      colorString = "Yellow";
    } else if (matchedColor == kYellowTarget) {

      colorString = "Green";

    } else {

      colorString = "Unknown";

    };

        /**

     * Open Smart Dashboard or Shuffleboard to see the color detected by the 

     * sensor.

     */

    frc::SmartDashboard::PutNumber("Red", detectedColor.red);

    frc::SmartDashboard::PutNumber("Green", detectedColor.green);

    frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);

    frc::SmartDashboard::PutNumber("Confidence", confidence);

    frc::SmartDashboard::PutString("Detected Color", colorString);

    frc::SmartDashboard::PutNumber("Proximity", m_colorSensor.GetProximity());

    frc::SmartDashboard::PutNumber("In Range", ControlPanel::InRange());

    frc::SmartDashboard::PutNumber("Rotations", turns);

    return colorString[0];
};

       ControlMotor::ControlMotor(){}
        TalonSRX srx = {13};
        
        bool ControlMotor::FastMotor(){
		
            bool flag;
            srx.Set(ControlMode::PercentOutput,.5);
            
            //Start control panel motors
            return flag;
        }

        bool ControlMotor::SlowMotor(){

            bool flag;
            srx.Set(ControlMode::PercentOutput,.20);
            
            //Start control panel motors
            return flag;
        }

        bool ControlMotor::StopMotor(){
            //Stop control panel motors
            bool flag;
			srx.Set(ControlMode::PercentOutput,0);
            return flag;
        }
        ControlMotor::~ControlMotor(){
		}
//-----------------------------------------------------------------------------------------------------------------
        ControlPanel::ControlPanel(){
		}
        ControlMotor CM;
        
        char done=0;
        //static char turns;
        bool colorflag;
        const char turn=7;
        bool ControlPanel::RotationControl(){
        
            //This section of code will count the number of times a particular is detected.

            /*if(ControlPanel::InRange()){
                //Read sensor range
                //if the sensor detects the 
                CM.FastMotor();

                //Count the number of occurrences of a color
                    //Break out of this routine after 4 revolutions of the control panel
                    
                    if(DetectColor()=='B' && ControlPanel::turns<turn && colorflag==1){
                        //count the number of times the color is detected
                        std::cout<< "turns="<< turns <<"\n";
                        ControlPanel::turns +1;
                        colorflag=0;
                        return 0;
                    }
                    else if(DetectColor()!='B' && turns<turn && colorflag==0)
                    {
                        std::cout<< "turns="<<ControlPanel::turns<<"\n";
                        colorflag=1;
                        return 0;
                    }
                    else if(ControlPanel::turns>turn -1){
                    CM.StopMotor();                 
                    ResetValues();
                    return 1;
                    }
            }*/
            CM.FastMotor();
                
        }
            bool ControlPanel::PositionControl(char TargetColor){

            //This section of code will detect the color that is 90 degress from sensed color.

            /*if(ControlPanel::InRange()){
               CM.SlowMotor();
               /*if (TargetColor==DetectColor()){
                   frc::SmartDashboard::PutString("Position Control", "Match");
                   //add a delay here
                   
                   CM.StopMotor();
                   
                   return 1;
               }
                else {
                    frc::SmartDashboard::PutString("Position Control", " No Match");
                    return 0;
                }
                */
               CM.SlowMotor();
            }
           /* switch (TargetColor)
            {
                case 'B' :
                //Blue case code
                CM.SlowMotor();
                //Sensor red
                break;
                case 'G' :
                //Green case code

                //Sensor yellow
                CM.FastMotor();
                break;
                case 'R' :
                //Red case code

                //Sensor blue
                CM.StopMotor();
                break;
                case 'Y' :
                //Yellow case code

                //Sensor green
                CM.StopMotor();
                break;
                default :
                //This is corrupt data
                break;
            }
            
               //Color detection code
               //CM.StopMotor();
               }
*/
        bool ControlPanel::InRange(){

    //A high value count respresents close proximity
            if (m_colorSensor.GetProximity()>800) {
                return 1;
            }
            else
            {
                CM.StopMotor();
                return 0;
            }
        }
        void ControlPanel::ResetValues(){
            CM.StopMotor();
            turns = 0;
            colorflag=0;
            }
		ControlPanel::~ControlPanel(){
		}
