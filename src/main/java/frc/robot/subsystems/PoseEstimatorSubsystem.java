package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
        this.gyro = new Pigeon2(25);

        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new edu.wpi.first.math.geometry.Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
            new edu.wpi.first.math.geometry.Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
            new edu.wpi.first.math.geometry.Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
            new edu.wpi.first.math.geometry.Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
        );

        // Initialize pose estimator
        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            Rotation2d.fromDegrees(gyro.getAngle()),
            getModulePositions(),
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, 0.01), 
            VecBuilder.fill(0.7, 0.7, 9999999) 
        );

        visionNotifier = new Notifier(this::updateVisionPose);
        visionNotifier.startPeriodic(0.1); //10 hz instead of 40
    }

    @Override
    public void periodic() {
        // Update odometry every cycle
        poseEstimator.update(Rotation2d.fromDegrees(gyro.getAngle()), getModulePositions());
    }

    private void updateVisionPose() {
        double currentTime = Timer.getFPGATimestamp();
        if (currentTime - lastVisionUpdate < 0.1) return; 

        lastVisionUpdate = currentTime;
        LimelightHelpers.PoseEstimate visionData = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

        if (visionData.tagCount > 0 && Math.abs(gyro.getRate()) <= 360) {
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
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
}
