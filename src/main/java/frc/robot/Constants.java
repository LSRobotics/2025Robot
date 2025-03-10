package frc.robot;
import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.Radian;

import java.util.function.BooleanSupplier;

public class Constants {
  public static class UniversalConstants {
    public static final String bestProgrammer = "Gabriel Kuzowsky"; 
  }

  public static class MultiversalConstants {
    public static final String bestProgrammer = "Ben Bell";
  }

  public static class GodConstants{
    public static final String bestProgrammer = "Michael Stauffer";
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static class SwerveSpeedConsts{
    public static final double slowSpeed = 0.5;
    public static final double L4Speed = 1d;
    public static final double L3Speed = 1d;
    public static final double L2Speed = 3d;
    public static final double L1Speed = 3d;
  }
  public static class LEDConstants{
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
    public static final double twinklesColorOneAndTwo = 0.51;
    public static final long matchTimeInMilliseconds = 150000;
  }

  public static class ClawConstants {
    public static final int clawMotorID = 31;
    public static final int sensorID = 62;
    public static final int gearRatio = 46;
    public static final int encoderTicksWithRatio = 2048 * gearRatio;
    public static final double manualClawSpeed = 0.05;
    public static final Angle L1ClawPosition = Angle.ofBaseUnits(0, Radian); 
  //public static final Angle L2L3ClawPosition = Angle.ofBaseUnits(0.4, Radian); 
    public static final Angle L2ClawPosition = Radian.of(0.4);
    public static final Angle L3ClawPosition = Radian.of(0.37);
  //public static final Angle L4ClawPosition = Angle.ofBaseUnits(0.75, Radian);
    public static final Angle L4ClawPosition = Angle.ofBaseUnits(1, Radian); // original 0.91 rad
    public static final Angle intermediateClawPos = Angle.ofBaseUnits(1.6, Radian);
    public static final Angle magicIntermediatedClawPos = Angle.ofBaseUnits(0.372, Radian);
    public static final Angle processorClawPos = Radian.of(3.25d); 
    public static final Angle algaeClawPos = Radian.of(2.8);
    public static final double limitClawPosition = 0;
    public static final double kP = 0.25;
    public static final double kD = 0.13;
    public static final double kG = 0.08; // 0.22
    public static final double kS = 0.0;
    public static final double kV = 0.91; // 0.88
    public static final double kA = 0.01;
    public static final double feedforwardVelocity = 1;
    public static final Angle tolerance = Angle.ofBaseUnits(0.05, Radian);
    public static final double GEAR_RATIO = 46;
    public static final double kSVoltage = -0.2;
    public static final double clampRangeforSpeed = 0.25;
  }
  public static class ShooterConstants {
    public static final int shooterMotorID = 30;
    public static final double intakeSpeed = -0.2;
    public static final double slowShooterSpeed = -0.4;
    public static final double algaeSpeed = 0.2;
    public static final double fastShooterSpeed = -0.6;
    public static final double slowAlgaeSpeed = 0.1;
    public static BooleanSupplier confirmAuton = ()->false;
  }
  public static class VisionConstants {
    public static final String LIMELIGHT_NAME = "";
    /* 
    public static final double MOVE_P = 0.300000;
    public static final double MOVE_I = 0.000000;
    public static final double MOVE_D = 0.000600;

    public static final double ROTATE_P = 0.030000;
    public static final double ROTATE_I = 0.000000;
    public static final double ROTATE_D = 0.000100;
    */

    public static final double X_REEF_ALIGNMENT_P = 0.15;
    public static final double Y_REEF_ALIGNMENT_P = 0.5;
    public static final double ROT_REEF_ALIGNMENT_P = 0.03;
    
    public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0;  
    public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 0.5;
    public static final double X_SETPOINT_REEF_ALIGNMENT = -0.5;  
    public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.005;
    public static final double Y_SETPOINT_REEF_ALIGNMENT = 0.4;  
    public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.0005;
  
    public static final double waitTime = 1;
    public static final double validationTime = 0.3;

    public static final double TOLERANCE = 0.01;
  }
  public static class ElevatorConstants {
    /** All values are placeholders for now*/
    public static final double kP = 0.75; // 0.03
    public static final double kI = 0.0;
    public static final double kD = 0.1; // 0.001 new 0.13

    public static final double kS = 0.24;
    public static final double kG = 1.01;
    public static final double kV = 10.52;
    public static final double kA = 0.05;
    public static final double feedforwardVelocity = 0.5;

    public static final double targetSpeed = 0.0025;
    public static final double elevatorPrecision = 1.0;
    public static final double maxVelocity = 0.1;
    public static final double maxAcceleration = 0.05;

    public static final double elevatorPosTolerance = 0.2;

    public static final double kElevatorMotorPort = 0;
    public static final int[] kEncoderPorts = {0, 1};
    public static final int kMotorID = 19;
    public static final int lMotorID = 20;

    public static final boolean kEncoderReversed = false;
    public static final double kEncoderDistancePerPulse = 1;
    public static final double kElevatorToleranceRPS = 100;

    /* 
    public static final double setpointLocation = 0.5;
    public static final double maxElevatorHeight = 1;
    public static final double level4HeightRatio = 0.9;
    public static final double algaePickupHighRatio = 0.53;
    public static final double level3HeightRatio = 0.5;
    public static final double algaePickupLowRatio = 0.23;
    public static final double level2HeightRatio = 0.2;
    public static final double coralPickupRatio = 0;
    */

    public static final double L1Height = 1.5d;
    //public static final double L2Height = 12d;
  //public static final double L2Height = 12d;
    public static final double L2Height = 11.5;
  //public static final double L3Height = 31d;
    public static final double L3Height = 29.75d;
    public static final double L4Height = 61d;
    public static final double A1Height = 22d; // original 20
    public static final double A2Height = 37d; // original 35
    public static final double processorHeight = 7d;  
    
    public static final double sprocketCircumefrence = 0.1397; //Meters
    public static final double clampRangeForSpeed = 6;
  }
}