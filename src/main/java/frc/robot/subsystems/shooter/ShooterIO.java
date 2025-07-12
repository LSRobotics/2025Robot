package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  public static class ShooterIOInputs {
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double motorTemperatureCelsius = 0.0;
    public double motorOutputPercent = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setSpeed(double percent) {}

  public default void stop() {}
}
