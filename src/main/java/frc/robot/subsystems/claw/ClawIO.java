package frc.robot.subsystems.claw;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Temperature;

public interface ClawIO {
    @AutoLog
    public static class ClawIOInputs {
        public Angle clawPosition = Radians.of(0d); // in radians
        public double clawVoltage = 0.0; // in volts
        public double clawMotorCurrent = 0.0; // in amps
        public double clawVelocity = 0.0; // in radians per second
        public double clawMotorAcceleration = 0.0; // in radians per second squared
        public Temperature clawMotorTemperature = Celsius.zero(); // in Celsius
    }

    public default void updateInputs(ClawIOInputs inputs) {
    }
    
    public default void setClawMotorSpeed(double speed) {
    }

    public default void setClawVoltage(double voltage) {
    }
} 