package frc.robot.subsystems.drive;

public class ModuleIOInputsAutoLogged {
    // Define the fields and methods for the class
    public double[] odometryTimestamps;
    public double[] odometryDrivePositionsRad;
    public Rotation2d[] odometryTurnPositions;
    public boolean driveConnected;
    public boolean turnConnected;
    public boolean turnEncoderConnected;
    public Rotation2d turnPosition;
    public double drivePositionRad;
    public double driveVelocityRadPerSec;

    public ModuleIOInputsAutoLogged() {
        // Initialize fields with default values
        odometryTimestamps = new double[0];
        odometryDrivePositionsRad = new double[0];
        odometryTurnPositions = new Rotation2d[0];
        driveConnected = true;
        turnConnected = true;
        turnEncoderConnected = true;
        turnPosition = new Rotation2d();
        drivePositionRad = 0.0;
        driveVelocityRadPerSec = 0.0;
    }
}