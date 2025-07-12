package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
  private Spark ledDriverOne;

  @Override
  public void periodic() {
    SmartDashboard.putNumber("LEDs", ledDriverOne.get());
  }

  public LEDSubsystem() {
    ledDriverOne = new Spark(Constants.LEDConstants.LEDDriverOneID);
  }

  public void runLEDs(double color) {
    ledDriverOne.set(color);
  }

  public double getLEDs(){
    return ledDriverOne.get();
  }
}