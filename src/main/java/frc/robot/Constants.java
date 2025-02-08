// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static final class ElevatorConstants {
    /** All values are placeholders for now*/
    public static final double kP = 0.5;
    public static final double kI = 0.01;
    public static final double kD = 0.1;
    public static final double elevatorPosTolerance = 1;
    public static final double elevatorVelTolerance = 1;
    public static final double kElevatorMotorPort = 0;
    public static final int[] kEncoderPorts = {0, 1};
    public static final boolean kEncoderReversed = false;
    public static final double kEncoderDistancePerPulse = 1;
    public static final double kS = 0.0;
    public static final double kG = 0.0;
    public static final double kV = 1.5;
    public static final double kA = 0.0;
    public static final double kElevatorToleranceRPS = 100;
    public static final int kMotorID = 19;
    public static final int lMotorID = 20;
    public static final double setpointLocation = -45000;
    public static final double maxElevatorHeight = 15000;
    public static final double level4HeightRatio = 0.9;
    public static final double algaePickupHighRatio = 0.53;
    public static final double level3HeightRatio = 0.5;
    public static final double algaePickupLowRatio = 0.23;
    public static final double level2HeightRatio = 0.2;
    public static final double coralPickupRatio = 0;
    public static final double targetSpeed = 0.010;
  } 
}
