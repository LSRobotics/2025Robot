package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    public double position = 0.0;
    public double velocityMetersPerSecond = 0.0;
    public double targetPositionMeters = 0.0;
    public boolean isZeroed = false;
    public boolean fastMode = false;
    public boolean limitSwitchTriggered = false;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double motorOutputPercent = 0.0;
    public double motorTemperatureCelsius = 0.0;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setVoltage(double volts) {}
  
  public default void setSpeed(double speed) {}

  public default void setEncoderPosition(double positionMeters) {}

  public default void zeroEncoder() {}

  public default boolean getElevatorLimitSwitch() {
    return false;
  }
}
