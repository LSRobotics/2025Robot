package frc.robot;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.Radian;

public class Constants {
  public static class UniversalConstants {
    public static final String bestProgrammer = "Gabriel Kuzowsky"; 
  }

  public static class MultiversalConstants {
    public static final String bestProgrammer = "Ben Bell";
  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
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
    public static final int clawMotorID = 31;
    public static final int shooterMotorID = 30;
    public static final int sensorID = 62;
    public static final int gearRatio = 46;
    public static final int encoderTicksWithRatio = 2048 * gearRatio;
    public static final double manualClawSpeed = 0.1;
    public static final double slowShooterSpeed = -0.3;
    public static final double fastShooterSpeed = -0.6;
    public static final double intakeSpeed = -0.2;
    public static final Angle L1ClawPosition = Angle.ofBaseUnits(0, Radian); 
    public static final Angle L2L3ClawPosition = Angle.ofBaseUnits(1, Radian); 
    public static final Angle L4ClawPosition = Angle.ofBaseUnits(56, Radian);
    public static final double algaeClawPosition = 100; // 270 degrees out of 2048 encoder ticks
    public static final double limitClawPosition = 0;
    public static final double kP = 0.1;
    public static final double kD = 0.15;
    public static final double tolerance = 0.1;
    public static final double GEAR_RATIO = 46;
  }

  public static class VisionConstants {
    public static final String LIMELIGHT_NAME = "";

    public static final double MOVE_P = 0.400000;
    public static final double MOVE_I = 0.000000;
    public static final double MOVE_D = 0.000600;

    public static final double ROTATE_P = 0.05000;
    public static final double ROTATE_I = 0.000000;
    public static final double ROTATE_D = 0.001000;

    public static final double TOLERANCE = 0.01;
  }
}
