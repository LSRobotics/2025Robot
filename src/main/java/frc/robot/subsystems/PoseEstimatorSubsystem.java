package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.Constants.PoseEstimationConstants;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.LimelightHelpers;

public class PoseEstimatorSubsystem extends SubsystemBase {
    private final CommandSwerveDrivetrain m_Swerve;
    private final Pigeon2 gyro;
    private final SwerveDrivePoseEstimator poseEstimator;

    private final Notifier visionNotifier;
    private double lastVisionUpdate = 0;

    public PoseEstimatorSubsystem(CommandSwerveDrivetrain swerve) {
        this.m_Swerve = swerve;
        this.gyro = new Pigeon2(PoseEstimationConstants.pigeonID);

        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            PoseEstimationConstants.frontLeftWheelLocation,
            PoseEstimationConstants.frontRightWheelLocation,
            PoseEstimationConstants.backLeftWheelLocation,
            PoseEstimationConstants.backRightWheelLocation
        );

        // Initialize pose estimator
        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            Rotation2d.fromDegrees(-gyro.getYaw().getValue().in(Degrees)),
            getModulePositions(),
            new Pose2d(),
            VecBuilder.fill(PoseEstimationConstants.odometryXSD, PoseEstimationConstants.odometryYSD, PoseEstimationConstants.odometryRotSD), 
            VecBuilder.fill(PoseEstimationConstants.visionXSD, PoseEstimationConstants.visionYSD, PoseEstimationConstants.visionRotSD) 
        );

        visionNotifier = new Notifier(this::updateVisionPose);
        visionNotifier.startPeriodic(PoseEstimationConstants.visionUpdatePeriod); //10 hz instead of 40
    }

    @Override
    public void periodic() {
        // Update odometry every cycle
        poseEstimator.update(Rotation2d.fromDegrees(-gyro.getYaw().getValue().in(Degrees)), getModulePositions());
    }

    private void updateVisionPose() {
        double currentTime = Timer.getFPGATimestamp();
        if (currentTime - lastVisionUpdate < 0.1) return; 

        lastVisionUpdate = currentTime;
        LimelightHelpers.PoseEstimate visionData = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

        if (visionData.tagCount > 0 && Math.abs(-gyro.getAngularVelocityZWorld().getValue().in(DegreesPerSecond)) <= 360) {
            VecBuilder.fill(PoseEstimationConstants.visionXSD, PoseEstimationConstants.visionYSD, PoseEstimationConstants.visionRotSD);
            poseEstimator.addVisionMeasurement(visionData.pose, visionData.timestampSeconds);
        }
    }

    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{
            m_Swerve.getModule(0).getPosition(true),
            m_Swerve.getModule(1).getPosition(true),
            m_Swerve.getModule(2).getPosition(true),
            m_Swerve.getModule(3).getPosition(true)
        };
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d newPose) {
        poseEstimator.resetPosition(
            Rotation2d.fromDegrees(-gyro.getYaw().getValue().in(Degrees)),
            getModulePositions(),
            newPose
        );
    }    
}
