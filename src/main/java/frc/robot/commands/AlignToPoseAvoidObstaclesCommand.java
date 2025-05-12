// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotation;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.units.Units;

/** An example command that uses an example subsystem. */
public class AlignToPoseAvoidObstaclesCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final CommandSwerveDrivetrain m_Swerve;
    private final PoseEstimatorSubsystem m_PoseEstimator;
    private final Pose2d targetPose;
    private final Command path;


    public AlignToPoseAvoidObstaclesCommand(CommandSwerveDrivetrain swerve, PoseEstimatorSubsystem poseEstimator,
            Pose2d targetPose) {
        m_Swerve = swerve;
        m_PoseEstimator = poseEstimator;
        this.targetPose = targetPose;

        PathConstraints constraints = new PathConstraints(
                3.0, 4.0,
                Radians.convertFrom(540, Degrees), Radians.convertFrom(720, Degrees));

        path = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0);

        addRequirements(swerve, poseEstimator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (!path.isScheduled()) {
            path.withTimeout(3.0).schedule(); //Prevent hang
        }
    }

    

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        path.cancel();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return path.isFinished();
    }
}