package frc.robot;

public class Constants {
  public static class UniversalConstants {
    public static final String bestProgrammer = "Gabriel Kuzowsky"; 
  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }
  public static class ElevatorConstants {
    public static final int kMotorID = 0;
    public static final double kMotorCircumference = 0;
    public static final double kP = 0.5;   // Strong correction
    public static final double kI = 0.01;  // Slight accumulation
    public static final double kD = 0.1;  // Moderate dampening
    public static final double kTolerance = 0.05;
  }
  public static class LEDConstants{
    public static final int LEDDriverOneID = 3;
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
    public static final double twinklesColorOneAndTwo = 0.51;
  }

  public static class ClawConstants {
    public static final int clawMotorID = 30;
    public static final int shooterMotorID = 31;
    public static final int sensorID = 62;
    public static final double manualClawSpeed = 0.2;
    public static final double slowShooterSpeed = 0.1;
    public static final double fastShooterSpeed = 0.6;
    public static final double L1ClawPosition = 15;
    public static final double L2L3ClawPosition = 45;
    public static final double L4ClawPosition = 90;
    public static final double algaeClawPosition = 270;
    public static final double kP = 0.003;
    public static final double kD = 0.08;
    public static final double tolerance = 2;
  }
}
