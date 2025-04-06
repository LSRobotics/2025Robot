package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.AlignConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import edu.wpi.first.math.MathUtil;

public class AlignToPoseCommand extends Command {
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController rotController;

  private final CommandSwerveDrivetrain mSwerve;
  private final PoseEstimatorSubsystem mPoseEstimator;
  private final Pose2d targetPose;

  private static final SwerveRequest.FieldCentric alignRequest = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

  public AlignToPoseCommand(CommandSwerveDrivetrain swerve, PoseEstimatorSubsystem poseEstimator, Pose2d targetPose) {
    this.mSwerve = swerve;
    this.mPoseEstimator = poseEstimator;
    this.targetPose = targetPose;

    xController = new PIDController(AlignConstants.xP, AlignConstants.xI, AlignConstants.xD);
    yController = new PIDController(AlignConstants.yP, AlignConstants.yI, AlignConstants.yD);
    rotController = new PIDController(AlignConstants.thetaP, AlignConstants.thetaI, AlignConstants.thetaD);

    rotController.enableContinuousInput(-180.0, 180.0);

    addRequirements(swerve, poseEstimator);
  }

  @Override
  public void initialize() {
    xController.setSetpoint(targetPose.getX());
    xController.setTolerance(AlignConstants.xTolerance);

    yController.setSetpoint(targetPose.getY());
    yController.setTolerance(AlignConstants.yTolerance);

    rotController.setSetpoint(targetPose.getRotation().getDegrees());
    rotController.setTolerance(AlignConstants.rotationTolerance);
  }

  @Override
  public void execute() {
    Pose2d currentPose = mPoseEstimator.getEstimatedPose();

    double xSpeed = xController.calculate(currentPose.getX());
    double ySpeed = yController.calculate(currentPose.getY());
    double rotSpeed = rotController.calculate(currentPose.getRotation().getDegrees());

    xSpeed = MathUtil.clamp(xSpeed, -AlignConstants.maxXSpeed, AlignConstants.maxXSpeed);
    ySpeed = MathUtil.clamp(ySpeed, -AlignConstants.maxYSpeed, AlignConstants.maxYSpeed);
    rotSpeed = MathUtil.clamp(rotSpeed, -AlignConstants.maxRotSpeed, AlignConstants.maxRotSpeed);

    SmartDashboard.putNumber("Align/Current X", currentPose.getX());
    SmartDashboard.putNumber("Align/Current Y", currentPose.getY());
    SmartDashboard.putNumber("Align/Current Rotation", currentPose.getRotation().getDegrees());

    mSwerve.setControl(alignRequest
        .withVelocityX(xSpeed)
        .withVelocityY(ySpeed)
        .withRotationalRate(rotSpeed));
  }

  @Override
  public void end(boolean interrupted) {
    mSwerve.setControl(idleRequest);
  }

  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint() && rotController.atSetpoint();
  }
}
