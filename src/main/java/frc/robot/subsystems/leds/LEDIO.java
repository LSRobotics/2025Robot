package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.AutoLog;

public interface LEDIO {
    @AutoLog
    public static class LEDIOInputs {
        public double ledColor = 0.0; // Color value for the LEDs
        public double ledVoltage = 0.0; // Voltage applied to the LEDs
        public boolean ledAlive = false; // Whether the LEDs are enabled
        public boolean pwmInverted = false; // Whether the PWM signal is inverted
    }

    public default void updateInputs(LEDIOInputs inputs) {
    }
    public default void setLEDColor(double color) {
    }
}
