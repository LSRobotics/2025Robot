package frc.robot.subsystems.claw;

import static edu.wpi.first.units.Units.Radian;

import edu.wpi.first.units.measure.Angle;

public class ClawConstants {
    public static final int clawMotorID = 31;
    public static final int sensorID = 62;
    public static final int gearRatio = 46;
    public static final int encoderTicksWithRatio = 2048 * gearRatio;
    public static final double manualClawSpeed = 0.05;
    public static final double bargeClawSpeed = 0.4; // 0.15
    public static final Angle L1ClawPosition = Angle.ofBaseUnits(0, Radian);
    // public static final Angle L2L3ClawPosition = Angle.ofBaseUnits(0.4, Radian);
    // public static final Angle L2ClawPosition = Radian.of(0.4);
    public static final Angle L2ClawPosition = Radian.of(0.6);
    public static final Angle L3ClawPosition = Radian.of(0.6);
    // public static final Angle L4ClawPosition = Angle.ofBaseUnits(0.75, Radian);
    public static final Angle L4ClawPosition = Angle.ofBaseUnits(1.05, Radian); // original 0.91 rad
    public static final Angle AutonL4Claw = Angle.ofBaseUnits(1.1, Radian);
    public static final Angle intermediateClawPos = Angle.ofBaseUnits(1.6, Radian);
    public static final Angle magicIntermediatedClawPos = Angle.ofBaseUnits(0.372, Radian);
    public static final Angle processorClawPos = Radian.of(3.25d);
    public static final Angle algaeClawPos = Radian.of(2.9);
    public static final Angle bargeClawPos = Radian.of(1.75); // 1.34
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
    public static final double kSVoltage = -0.15;
    public static final double clampRangeforSpeed = 0.25;
}
