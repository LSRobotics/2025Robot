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

  public static class VisionConstants {
    public static final String LIMELIGHT_NAME = "";
    public static final double LIMELIGHT_LENS_HEIGHT = 10; //Find later
    public static final double LIMELIGHT_ANGLE = 30;//Find later

    public static final double REEF_APRILTAG_HEIGHT = 6.875;
    public static final double PROCCESSOR_APRILTAG_HEIGHT = 45.875;
    public static final double CORAL_APRILTAG_HEIGHT = 53.25;
  }
}
