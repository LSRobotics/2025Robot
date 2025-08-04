package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {

  public static class LEDConstants {
    public static final int LEDDriverOneID = 1;
    public static final double colorRed = 0.61;
    public static final double colorHotPink = 0.57;
    public static final double colorYellow = 0.69;
    public static final double colorSkyBlue = 0.83;
    public static final double colorBlueViolet = 0.89;
    public static final double colorWhite = 0.93;
    public static final double colorLimeGreen = 0.73;
    public static final double colorOrange = 0.65;
    public static final double colorDarkGreen = 0.75;
    public static final double colorLawnGreen = 0.71;
    public static final double colorBlue = 0.87;
    public static final double colorGold = 0.67;
    public static final double colorOff = 0.0;
    public static final double twinklesColorOneAndTwo = 0.51;
    public static final long matchTimeInMilliseconds = 150_000;

    public enum PWMToHexColor {
      RED(0.61, "#FF0000"),
      HOT_PINK(0.57, "#FF69B4"),
      YELLOW(0.69, "#FFFF00"),
      SKY_BLUE(0.83, "#87CEEB"),
      BLUE_VIOLET(0.89, "#8A2BE2"),
      WHITE(0.93, "#FFFFFF"),
      LIME_GREEN(0.73, "#32CD32"),
      ORANGE(0.65, "#FFA500"),
      DARK_GREEN(0.75, "#006400"),
      LAWN_GREEN(0.71, "#7CFC00"),
      BLUE(0.87, "#0000FF"),
      GOLD(0.67, "#FFD700");

      private final double pwmValue;
      private final String hexColor;

      PWMToHexColor(double pwmValue, String hexColor) {
        this.pwmValue = pwmValue;
        this.hexColor = hexColor;
      }

      public double getPWMValue() {
        return pwmValue;
      }

      public String getHexColor() {
        return hexColor;
      }

      public static String getHexColorFromPWM(double pwmValue) {
        final double TOLERANCE = 0.001; 
        for (PWMToHexColor color : values()) {
          if (Math.abs(color.getPWMValue() - pwmValue) < TOLERANCE) {
            return color.getHexColor();
          }
        }
        return "#000000"; 
      }
    }
  }

  private final Spark ledDriverOne;

  private boolean isPulsing = false;
  private long pulseStartTime = 0;
  private long pulseDurationMs = 0;
  private double pulseOffColor = 0.0;
  private double previousColor = LEDConstants.colorWhite;
  private boolean usePreviousColor = true;

  public LEDSubsystem() {
    ledDriverOne = new Spark(LEDConstants.LEDDriverOneID);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("LEDs", ledDriverOne.get());
    SmartDashboard.putBoolean("LEDs/IsPulsing", isPulsing);
    SmartDashboard.putNumber("LEDs/PulseDuration", pulseDurationMs);
    SmartDashboard.putNumber("LEDs/PreviousColor", previousColor);
    SmartDashboard.putBoolean("LEDs/UsePreviousColor", usePreviousColor);
    SmartDashboard.putString("Current Color", LEDConstants.PWMToHexColor.getHexColorFromPWM(getLEDs()));
    
    if (isPulsing) {
      long now = System.currentTimeMillis();
      long timeRemaining = Math.max(0, pulseDurationMs - (now - pulseStartTime));
      SmartDashboard.putNumber("LEDs/PulseTimeRemaining", timeRemaining);
      
      if (now - pulseStartTime >= pulseDurationMs) {
        isPulsing = false;
        runLEDs(usePreviousColor ? previousColor : pulseOffColor);
      }
    } else {
      SmartDashboard.putNumber("LEDs/PulseTimeRemaining", 0);
    }
  }

  public void runLEDs(double color) {
    SmartDashboard.putString("LEDs/LastAction", "Set Color");
    SmartDashboard.putNumber("LEDs/SetColor", color);
    ledDriverOne.set(color);
  }
  

  public double getLEDs() {
    return ledDriverOne.get();
  }

  // Flash on color, then return to previous color
  public void blink(long durationMs, double onColor) {
    SmartDashboard.putString("LEDs/LastAction", "Blink (return to previous)");
    SmartDashboard.putNumber("LEDs/BlinkDuration", durationMs);
    SmartDashboard.putNumber("LEDs/BlinkOnColor", onColor);
    
    this.pulseDurationMs = durationMs;
    this.previousColor = getLEDs();
    this.usePreviousColor = true;
    this.isPulsing = true;
    this.pulseStartTime = System.currentTimeMillis();
    runLEDs(onColor);
  }

  public void blink(long durationMs, double onColor, double offColor) {
    SmartDashboard.putString("LEDs/LastAction", "Blink (with off color)");
    SmartDashboard.putNumber("LEDs/BlinkDuration", durationMs);
    SmartDashboard.putNumber("LEDs/BlinkOnColor", onColor);
    SmartDashboard.putNumber("LEDs/BlinkOffColor", offColor);
    
    this.pulseDurationMs = durationMs;
    this.pulseOffColor = offColor;
    this.usePreviousColor = false;
    this.isPulsing = true;
    this.pulseStartTime = System.currentTimeMillis();
    runLEDs(onColor);
  }
}
