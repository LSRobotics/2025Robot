package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.inputs.LoggableInputs;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.LogTable;

public class ModuleIOInputsAutoLogged extends ModuleIO.ModuleIOInputs implements LoggableInputs {
    @Override
    public void toLog(LogTable table) {
        // Implement the method to log data to the table
    }

    @Override
    public void fromLog(LogTable table) {
        // Implement the method to read data from the table
    }
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