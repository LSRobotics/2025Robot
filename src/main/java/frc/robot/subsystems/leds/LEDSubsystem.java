package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  private final LEDIO io;
  private final LEDIOInputsAutoLogged inputs = new LEDIOInputsAutoLogged();

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("LED", inputs);
  }

  public LEDSubsystem(LEDIO io) {
    this.io=io;
  }

  public void runLEDs(double color) {
    io.setLEDColor(color);
  }

  public double getLEDs(){
    return inputs.ledColor;
  }
}