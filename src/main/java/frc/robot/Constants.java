package frc.robot;
import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.Radian;

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
    public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.1;
  
    public static final double waitTime = 1;
    public static final double validationTime = 0.3;

    public static final double branchAngle = 21d; //Degrees
    public static final double branchTolerance = 2.5; //Degrees

    public static final double AlgaeAngle = 0d; //Degrees
    public static final double AlgaeTolerance = 3.5d; //Degrees

    public static final double TOLERANCE = 0.01;

    public static final double alignSpeed = 0.35;
  }

}