package frc.robot.subsystems.leds;
import edu.wpi.first.wpilibj.DriverStation;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class LEDIOBlinken implements LEDIO {
    private Spark blinken = new Spark(LEDConstants.LEDDriverOneID);

    public LEDIOBlinken() {
    }

    @Override
    public void updateInputs(LEDIOInputs inputs) {
        inputs.ledColor = blinken.get();
        inputs.ledVoltage = blinken.getVoltage();
        inputs.ledAlive = blinken.isAlive();
        inputs.pwmInverted = blinken.getInverted(); 
    }

    @Override
    public void setLEDColor(double color) {
        if (color < -1d || color > 1d) {
            DriverStation.reportWarning("PWM color value must be between -1 and 1, inclusive. Provided: " + color, false);
        }
        blinken.set(color);
    }
}
