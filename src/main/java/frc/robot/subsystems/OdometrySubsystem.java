package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.Encoder;

public class OdometrySubsystem {
    // Locations for the swerve drive modules relative to the robot center.
    Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
    Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);
    // Creating my kinematics object using the module locations
    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    );
    SwerveDriveKinematics SwerveDriveKinematics = new SwerveDriveKinematics();
    // Example chassis speeds: 1 meter per second forward, 3 meters
    // per second to the left, and rotation at 1.5 radians per second
    // counterclockwise.
    ChassisSpeeds Cspeeds = new ChassisSpeeds(1.0, 3.0, 1.5);
    // Convert to module states
    SwerveModuleState[] SmoduleStates = SwerveDriveKinematics.toSwerveModuleStates(Cspeeds);
    // Front left module state
    SwerveModuleState frontLeft = SmoduleStates[0];
    // Front right module state
    SwerveModuleState frontRight = SmoduleStates[1];
    // Back left module state
    SwerveModuleState backLeft = SmoduleStates[2];
    // Back right module state
    SwerveModuleState backRight = SmoduleStates[3];

    private Encoder m_turningEncoder = new Encoder(null, null);

    SwerveModuleState CfrontLeftOptimized = SwerveModuleState.optimize(frontLeft,
    new Rotation2d(m_turningEncoder.getDistance()));

   Rotation2d currentAngle = new Rotation2d(m_turningEncoder.getDistance());
    SwerveModuleState frontLeftOptimized = SwerveModuleState.optimize(frontLeft, currentAngle);
    frontLeftOptimized = new SwerveModuleState(
        frontLeftOptimized.speedMetersPerSecond * CfrontLeftOptimized.angle.minus(currentAngle).getCos(),
        frontLeftOptimized.angle
    );

    // The desired field relative speed here is 2 meters per second
    // toward the opponent's alliance station wall, and 2 meters per
    // second toward the left field boundary. The desired rotation
    // is a quarter of a rotation per second counterclockwise. The current
    // robot angle is 45 degrees.
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
    2.0, 2.0, Math.PI / 2.0, Rotation2d.fromDegrees(45.0));
    // Now use this in our kinematics
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

    // Example module states
    SwerveModuleState frontLeftState = new SwerveModuleState(23.43, Rotation2d.fromDegrees(-140.19));
    SwerveModuleState frontRightState = new SwerveModuleState(23.43, Rotation2d.fromDegrees(-39.81));
    SwerveModuleState backLeftState = new SwerveModuleState(54.08, Rotation2d.fromDegrees(-109.44));
    SwerveModuleState backRightState = new SwerveModuleState(54.08, Rotation2d.fromDegrees(-70.56));
    // Convert to chassis speeds
    ChassisSpeeds chassisSpeeds = m_kinematics.toChassisSpeeds(
    frontLeftState, frontRightState, backLeftState, backRightState);
    // Getting individual speeds
    double forward = chassisSpeeds.vxMetersPerSecond;
    double sideways = chassisSpeeds.vyMetersPerSecond;
    double angular = chassisSpeeds.omegaRadiansPerSecond;

    public class Example {
  private final StructArrayPublisher<SwerveModuleState> publisher;
  public Example() {
    // Start publishing an array of module states with the "/SwerveStates" key
    publisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();
  }
  public void periodic() {
    // Periodically send a set of module states
    publisher.set(new SwerveModuleState[] {
      frontLeftState,
      frontRightState,
      backLeftState,
      backRightState
    });
  }
}
}
