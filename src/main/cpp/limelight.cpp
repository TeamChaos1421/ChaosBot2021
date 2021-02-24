#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

//limelight variables
  float targetX;
  float targetY;
  float targetA;
  std::shared_ptr<NetworkTable> table;
  double targetSpeed=0.4;
  double fineTargetSpeed=0.35;
  int targetfoundCounter=0;
  
  table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");


int limeLight() {
	double turn;
	targetX = table->GetNumber("tx", 0);
    targetY = table->GetNumber("ty", 0);
    targetA = table->GetNumber("ta", 0);
    wpi::outs() << "target X is  " << targetX << "\n";
    wpi::outs() << "target Y is  " << targetY << "\n";
    wpi::outs() << "target A is  " << targetA << "\n";
	
	if (targetX > 1) {
		turn = targetSpeed;
	}
	else if (targetX > 0.5) {
		turn = fineTargetSpeed;
	}
	else if (targetX < -1.0) {
		turn = -(targetSpeed);
	}
	else if (targetX < -0.5) {
		turn = -(fineTargetSpeed);
	}
	//else if ((targetX < 0.5) & (targetX > -0.5)) {
		//turn = 0;
	//}
	else {
		turn = 0;
	}
	
	return turn;
}