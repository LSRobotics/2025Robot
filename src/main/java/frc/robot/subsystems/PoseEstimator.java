package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.Constants.PoseEstimationConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.LimelightHelpers.RawFiducial;

/**
 * Simplified PoseEstimator that delegates pose estimation to the CommandSwerveDrivetrain
 * and only handles vision measurement integration.
 */
public class PoseEstimator {
    private static CommandSwerveDrivetrain m_Swerve;
    private static Pigeon2 gyro;
    
    private static Notifier visionNotifier;
    private static double lastVisionUpdate = 0;
    
    private static boolean isInitialized = false;

    private PoseEstimator() {
        throw new IllegalStateException("use staticlly"); //probaly should be assertionErr
    }

    public static void initialize(CommandSwerveDrivetrain swerve) {
        if (isInitialized) {
            DriverStation.reportWarning("Pose Estimator initalizede  twice", false);
            return; //prevent multipoe instantionatins
        }
        
        m_Swerve = swerve;
        gyro = new Pigeon2(PoseEstimationConstants.pigeonID);

        visionNotifier = new Notifier(PoseEstimator::updateVisionPose);
        visionNotifier.startPeriodic(PoseEstimationConstants.visionUpdatePeriod); // 10 hz instead of 40 to avoid loop overruns
        
        isInitialized = true;
    }

    public static void periodic() {
        ensureInitialized();
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

        // microsecs ot secs and handles latenceyc
        double adjustedTimestamp = (timestamp / 1000000.0) - (latency / 1000.0);

        RawFiducial[] rawFiducials = new RawFiducial[tagCount];
        int valsPerFiducial = 7;
        int expectedTotalVals = 11 + valsPerFiducial * tagCount;

        if (poseArray.length != expectedTotalVals) {
            // silently fail
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
            return; // avoid redundant updates

        lastVisionUpdate = currentTime;
        LimelightHelpers.PoseEstimate visionData = getVisionEstimate("limelight", "botpose_wpiblue", true);

        if (visionData != null && visionData.tagCount > 0
                && Math.abs(-gyro.getAngularVelocityZWorld().getValue().in(DegreesPerSecond)) <= 360) {
            
            Matrix<N3, N1> visionStdDevs = createDynamicVisionStdDevs(visionData);
            
            m_Swerve.addVisionMeasurement(visionData.pose, visionData.timestampSeconds, visionStdDevs); //TODO: check time formatd 
        }
    }


    private static Matrix<N3, N1> createDynamicVisionStdDevs(PoseEstimate visionData) {
        double baseXStd = PoseEstimationConstants.visionXSD;
        double baseYStd = PoseEstimationConstants.visionYSD;
        double baseRotStd = PoseEstimationConstants.visionRotSD;
        
        // more tags = more confidencee
        double tagCountMultiplier = Math.max(0.5, 2.0 / visionData.tagCount);
        
        // farther distance is less confidence
        double distanceMultiplier = Math.max(1.0, visionData.avgTagDist / 4.0);
        
        // wider span= morre confidencee
        double spanMultiplier = Math.max(0.8, 2.0 / Math.max(visionData.avgTagArea, 0.1));
        
        double finalMultiplier = tagCountMultiplier * distanceMultiplier * spanMultiplier;
        
        return VecBuilder.fill(
            baseXStd * finalMultiplier,
            baseYStd * finalMultiplier,
            baseRotStd * finalMultiplier
        );
    }

    public static Pose2d getEstimatedPose() {
        ensureInitialized();
        return m_Swerve.getState().Pose;
    }

    public static void resetPose(Pose2d newPose) {
        ensureInitialized();
        m_Swerve.resetPose(newPose);
    }

    public static edu.wpi.first.math.kinematics.SwerveDriveKinematics getKinematics() {
        ensureInitialized();

        return m_Swerve.getKinematics();
    }
    

    public static edu.wpi.first.math.kinematics.ChassisSpeeds getCurrentSpeeds() {
        ensureInitialized();
        return m_Swerve.getState().Speeds;
    }
    

    public static SwerveModulePosition[] getModulePositions() {
        ensureInitialized();
        return new SwerveModulePosition[] {
            m_Swerve.getModule(0).getPosition(true),
            m_Swerve.getModule(1).getPosition(true),
            m_Swerve.getModule(2).getPosition(true),
            m_Swerve.getModule(3).getPosition(true)
        };
    }

    public static SwerveModuleState[] getModuleStates() {
        ensureInitialized();
        return new SwerveModuleState[] {
            m_Swerve.getModule(0).getCurrentState(),
            m_Swerve.getModule(1).getCurrentState(),
            m_Swerve.getModule(2).getCurrentState(),
            m_Swerve.getModule(3).getCurrentState()
        };
    }
    
    private static void ensureInitialized() {
        if (!isInitialized) {
            throw new IllegalStateException("PoseEstimatorSubsystem must be initialized before use. Call initialize() first.");
        }
    }
    
    public static void shutdown() {
        if (visionNotifier != null) {
            visionNotifier.stop();
        }
        isInitialized = false;
    }
}