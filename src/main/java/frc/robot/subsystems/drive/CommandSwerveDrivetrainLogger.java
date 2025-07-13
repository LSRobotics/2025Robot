package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Radians;

import java.util.Arrays;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class CommandSwerveDrivetrainLogger {
    private static Notifier notifier;
    private static CommandSwerveDrivetrain drivetrain;
    private static String subsystemName;

    public static void start(String name, CommandSwerveDrivetrain swerveDrivetrain, double period) {

        subsystemName = name;
        drivetrain = swerveDrivetrain;

        dashboardInit();

        if (notifier == null) {
            notifier = new Notifier(CommandSwerveDrivetrainLogger::periodicLog);
        }
        notifier.startPeriodic(period);
    }

    public static void dashboardInit(){
        SmartDashboard.putData("Swerve Drive", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                builder.addDoubleProperty("Front Left Angle", () -> drivetrain.getModule(0).getCurrentState().angle.getRadians(), null);
                builder.addDoubleProperty("Front Left Velocity", () -> drivetrain.getModule(0).getCurrentState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Front Right Angle", () -> drivetrain.getModule(1).getCurrentState().angle.getRadians(), null);
                builder.addDoubleProperty("Front Right Velocity", () -> drivetrain.getModule(1).getCurrentState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Back Left Angle", () -> drivetrain.getModule(2).getCurrentState().angle.getRadians(), null);
                builder.addDoubleProperty("Back Left Velocity", () -> drivetrain.getModule(2).getCurrentState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Back Right Angle", () -> drivetrain.getModule(3).getCurrentState().angle.getRadians(), null);
                builder.addDoubleProperty("Back Right Velocity", () -> drivetrain.getModule(3).getCurrentState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Robot Angle", () -> drivetrain.getPigeon2().getYaw().getValue().in(Radians), null);
            }
        });
    }


    public static void stop() {
        if (notifier != null) {
            notifier.stop();
            notifier.close();
            notifier = null;
        }

    }

    private static void periodicLog() {
        log(subsystemName, drivetrain);
    }

    public static void log(String subsystemName, CommandSwerveDrivetrain drivetrain) {
        String path = subsystemName + "/";

        Logger.recordOutput(path + "Pose", drivetrain.getState().Pose);

        ChassisSpeeds speeds = drivetrain.getState().Speeds;
        Logger.recordOutput(path + "Chassis Speed", speeds);
        Logger.recordOutput(path + "ChassisSpeedX", speeds.vxMetersPerSecond);
        Logger.recordOutput(path + "ChassisSpeedY", speeds.vyMetersPerSecond);
        Logger.recordOutput(path + "ChassisSpeedOmega", speeds.omegaRadiansPerSecond);

        SwerveModuleState[] states = drivetrain.getState().ModuleStates;
        Logger.recordOutput(path + "ModuleStates", states);

        SwerveModulePosition[] positions = drivetrain.getState().ModulePositions;
        Logger.recordOutput(path + "ModulePositions", positions);

        SwerveModuleState[] desiredStates = drivetrain.getState().ModuleTargets;
        Logger.recordOutput(path + "DesiredModuleStates", desiredStates);

        for (int i = 0; i < states.length; i++) {
            String modulePath = path + "Module" + i + "/";
            Logger.recordOutput(modulePath + "Angle", states[i].angle.getDegrees());
            Logger.recordOutput(modulePath + "Speed", states[i].speedMetersPerSecond);
            Logger.recordOutput(modulePath + "Position", positions[i].distanceMeters);
            Logger.recordOutput(modulePath + "DesiredAngle", desiredStates[i].angle.getDegrees());
            Logger.recordOutput(modulePath + "DesiredSpeed", desiredStates[i].speedMetersPerSecond);

            var driveMotor = drivetrain.getModule(i).getDriveMotor();
            String driveMotorPath = modulePath + "DriveMotor/";
            Logger.recordOutput(driveMotorPath + "Voltage", driveMotor.getMotorVoltage().getValue());
            Logger.recordOutput(driveMotorPath + "CurrentAmps", driveMotor.getSupplyCurrent().getValue());
            Logger.recordOutput(driveMotorPath + "TempCelsius", driveMotor.getDeviceTemp().getValue());

            var steerMotor = drivetrain.getModule(i).getSteerMotor();
            String steerMotorPath = modulePath + "SteerMotor/";
            Logger.recordOutput(steerMotorPath + "Voltage", steerMotor.getMotorVoltage().getValue());
            Logger.recordOutput(steerMotorPath + "CurrentAmps", steerMotor.getSupplyCurrent().getValue());
            Logger.recordOutput(steerMotorPath + "TempCelsius", steerMotor.getDeviceTemp().getValue());
        }

        // Voltage and current for each motor
        Logger.recordOutput(path + "Operator Forward", drivetrain.getOperatorForwardDirection());

        Logger.recordOutput(path + "Odometry Period", drivetrain.getState().OdometryPeriod);

        Pigeon2 pigeon = drivetrain.getPigeon2();
        String pigeonPath = path + "Pigeon/";
        Logger.recordOutput(pigeonPath + "Yaw", pigeon.getYaw().getValue());
        Logger.recordOutput(pigeonPath + "Pitch", pigeon.getPitch().getValue());
        Logger.recordOutput(pigeonPath + "Roll", pigeon.getRoll().getValue());
        Logger.recordOutput(pigeonPath + "AngularVelocityX", pigeon.getAngularVelocityXWorld().getValue());
        Logger.recordOutput(pigeonPath + "AngularVelocityY", pigeon.getAngularVelocityYWorld().getValue());
        Logger.recordOutput(pigeonPath + "AngularVelocityZ", pigeon.getAngularVelocityZWorld().getValue());
        Logger.recordOutput(pigeonPath + "AccelerationX", pigeon.getAccelerationX().getValue());
        Logger.recordOutput(pigeonPath + "AccelerationY", pigeon.getAccelerationY().getValue());
        Logger.recordOutput(pigeonPath + "AccelerationZ", pigeon.getAccelerationZ().getValue());
    }
}
