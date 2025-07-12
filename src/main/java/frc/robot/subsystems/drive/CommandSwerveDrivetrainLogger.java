package frc.robot.subsystems.drive;

import java.util.Arrays;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class CommandSwerveDrivetrainLogger {
    private static Notifier notifier;
    private static CommandSwerveDrivetrain drivetrain;
    private static String subsystemName;

    public static void start(String name, CommandSwerveDrivetrain swerveDrivetrain, double period) {
        subsystemName = name;
        drivetrain = swerveDrivetrain;
        if (notifier == null) {
            notifier = new Notifier(CommandSwerveDrivetrainLogger::periodicLog);
        }
        notifier.startPeriodic(period);
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
        }

        //Voltage and current for each motor
        Logger.recordOutput(path+"Operator Forward", drivetrain.getOperatorForwardDirection());

        Logger.recordOutput(path+"Odometry Period", drivetrain.getState().OdometryPeriod);

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
