package frc.robot.subsystems.elevator;

public class ElevatorConstants {
    /** All values are placeholders for now */
    public static final double kP = 0.6; // 0.03
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
    public static final int[] kEncoderPorts = { 0, 1 };
    public static final int kMotorID = 19;
    public static final int lMotorID = 20;

    public static final boolean kEncoderReversed = false;
    public static final double kEncoderDistancePerPulse = 1;
    public static final double kElevatorToleranceRPS = 100;

    public static final double elevatorHoldVoltage = 0.38;
    public static final double elevatorManualHoldVoltage = 0.1;

    /*
     * public static final double setpointLocation = 0.5;
     * public static final double maxElevatorHeight = 1;
     * public static final double level4HeightRatio = 0.9;
     * public static final double algaePickupHighRatio = 0.53;
     * public static final double level3HeightRatio = 0.5;
     * public static final double algaePickupLowRatio = 0.23;
     * public static final double level2HeightRatio = 0.2;
     * public static final double coralPickupRatio = 0;
     */

    public static final double L1Height = 0.7;
    // public static final double L2Height = 12d;
    // public static final double L2Height = 12d;
    // public static final double L2Height = 11.5;
    public static final double L2Height = 13d;
    // public static final double L3Height = 31d;
    public static final double L3Height = 31d;
    public static final double L4Height = 60.5;
    public static final double A1Height = 22d; // original 20
    public static final double A2Height = 40d; // original 35
    public static final double processorHeight = 7d;
    public static final double bargeHeight = 63d;

    public static final double sprocketCircumefrence = 0.1397; // Meters
    public static final double clampRangeForSpeed = 5;
}