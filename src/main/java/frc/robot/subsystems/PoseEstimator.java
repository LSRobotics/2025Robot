package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.Constants.PoseEstimationConstants;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.LimelightHelpers.RawFiducial;

public class PoseEstimator {
    private static CommandSwerveDrivetrain m_Swerve;
    private static Pigeon2 gyro;
    private static SwerveDrivePoseEstimator poseEstimator;

    private static Notifier visionNotifier;
    private static double lastVisionUpdate = 0;

    private static SwerveDriveKinematics kinematics;
    private static boolean isInitialized = false;

    // Private constructor to prevent instantiation
    private PoseEstimator() {}

    public static void initialize(CommandSwerveDrivetrain swerve) {
        if (isInitialized) {
            return; // Prevent multiple initializations
        }
        
        m_Swerve = swerve;
        gyro = new Pigeon2(PoseEstimationConstants.pigeonID);

        kinematics = new SwerveDriveKinematics(
                PoseEstimationConstants.frontLeftWheelLocation,
                PoseEstimationConstants.frontRightWheelLocation,
                PoseEstimationConstants.backLeftWheelLocation,
                PoseEstimationConstants.backRightWheelLocation);

        poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                Rotation2d.fromDegrees(-gyro.getYaw().getValue().in(Degrees)),
                getModulePositions(),
                new Pose2d(),
                VecBuilder.fill(PoseEstimationConstants.odometryXSD, PoseEstimationConstants.odometryYSD,
                        PoseEstimationConstants.odometryRotSD),
                VecBuilder.fill(PoseEstimationConstants.visionXSD, PoseEstimationConstants.visionYSD,
                        PoseEstimationConstants.visionRotSD));

        visionNotifier = new Notifier(PoseEstimator::updateVisionPose);
        visionNotifier.startPeriodic(PoseEstimationConstants.visionUpdatePeriod); // 10 hz instead of 40 to avoid loop overruns
        
        isInitialized = true;
    }

    public static void periodic() { // Update with odometry data
        ensureInitialized();
        poseEstimator.update(Rotation2d.fromDegrees(-gyro.getYaw().getValue().in(Degrees)), getModulePositions());
    }

    private static double getEntry(double[] data, int position) {
        if (data.length < position + 1) {
            return 0;
        }
        return data[position];
    }

    private static PoseEstimate getVisionEstimate(String limelightName, String entryName, boolean isMegaTag2) {
        DoubleArrayEntry poseEntry = LimelightHelpers.getLimelightDoubleArrayEntry(limelightName, entryName);

        TimestampedDoubleArray tsValue = poseEntry.getAtomic();
        double[] poseArray = tsValue.value;
        long timestamp = tsValue.timestamp;

        if (poseArray.length == 0) {
            return null; 
        }

        Pose2d pose;
        if (poseArray.length < 6) {
            pose = new Pose2d();
            DriverStation.reportWarning("LL pose array missing elements", false);
        }
        else {
            Translation2d tran2d = new Translation2d(poseArray[0], poseArray[1]);
            Rotation2d r2d = new Rotation2d(Units.degreesToRadians(poseArray[5]));
            pose = new Pose2d(tran2d, r2d);
        }

        double latency = getEntry(poseArray, 6);
        int tagCount = (int) getEntry(poseArray, 7);
        double tagSpan = getEntry(poseArray, 8);
        double tagDist = getEntry(poseArray, 9);
        double tagArea = getEntry(poseArray, 10);

        // Microseconds to seconds and handle latency
        double adjustedTimestamp = (timestamp / 1000000.0) - (latency / 1000.0);

        RawFiducial[] rawFiducials = new RawFiducial[tagCount];
        int valsPerFiducial = 7;
        int expectedTotalVals = 11 + valsPerFiducial * tagCount;

        if (poseArray.length != expectedTotalVals) {
            // Handle error case silently
        } 
        else {
            for (int i = 0; i < tagCount; i++) {
                int baseIndex = 11 + (i * valsPerFiducial);
                int id = (int) poseArray[baseIndex];
                double txnc = poseArray[baseIndex + 1];
                double tync = poseArray[baseIndex + 2];
                double ta = poseArray[baseIndex + 3];
                double distToCamera = poseArray[baseIndex + 4];
                double distToRobot = poseArray[baseIndex + 5];
                double ambiguity = poseArray[baseIndex + 6];
                rawFiducials[i] = new RawFiducial(id, txnc, tync, ta, distToCamera, distToRobot, ambiguity);
            }
        }

        return new PoseEstimate(pose, adjustedTimestamp, latency, tagCount, tagSpan, tagDist, tagArea, rawFiducials,
                isMegaTag2);
    }

    private static void updateVisionPose() {
        ensureInitialized();
        double currentTime = Timer.getFPGATimestamp();
        if (currentTime - lastVisionUpdate < 0.1)
            return; // Avoid updating twice

        lastVisionUpdate = currentTime;
        LimelightHelpers.PoseEstimate visionData = getVisionEstimate("limelight", "botpose_wpiblue", true);

        if (visionData != null && visionData.tagCount > 0
                && Math.abs(-gyro.getAngularVelocityZWorld().getValue().in(DegreesPerSecond)) <= 360) {
            VecBuilder.fill(PoseEstimationConstants.visionXSD, PoseEstimationConstants.visionYSD,
                    PoseEstimationConstants.visionRotSD);
            poseEstimator.addVisionMeasurement(visionData.pose, visionData.timestampSeconds);
        }
    }

    private static SwerveModulePosition[] getModulePositions() {
        ensureInitialized();
        return new SwerveModulePosition[] {
                m_Swerve.getModule(0).getPosition(true),
                m_Swerve.getModule(1).getPosition(true),
                m_Swerve.getModule(2).getPosition(true),
                m_Swerve.getModule(3).getPosition(true)
        };
    }

    public static Pose2d getEstimatedPose() {
        ensureInitialized();
        return poseEstimator.getEstimatedPosition();
    }

    public static void resetPose(Pose2d newPose) {
        ensureInitialized();
        poseEstimator.resetPosition(
                Rotation2d.fromDegrees(-gyro.getYaw().getValue().in(Degrees)),
                getModulePositions(),
                newPose);
    }

    public static SwerveDriveKinematics getKinematics() {
        ensureInitialized();
        return kinematics;
    }
    
    // Helper method to ensure the subsystem is initialized before use
    private static void ensureInitialized() {
        if (!isInitialized) {
            throw new IllegalStateException("PoseEstimatorSubsystem must be initialized before use. Call initialize() first.");
        }
    }
    
    // Clean up resources when the robot is disabled/stopped
    public static void shutdown() {
        if (visionNotifier != null) {
            visionNotifier.stop();
        }
        // Add any other cleanup needed
    }
}