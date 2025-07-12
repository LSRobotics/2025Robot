// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AutonAlgaeAlignCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final VisionSubsystem m_Vision;
    private final CommandSwerveDrivetrain m_Swerve;
    private final double setpoint;
    private static final SwerveRequest.RobotCentric alignRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public AutonAlgaeAlignCommand(VisionSubsystem vision, CommandSwerveDrivetrain swerve) {
        m_Vision = vision;
        m_Swerve = swerve;

        setpoint = VisionConstants.AlgaeAngle;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(vision, swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_Vision.getTX() < setpoint) {
            m_Swerve.setControl(alignRequest.withVelocityY(-VisionConstants.alignSpeed));
        } 
        else if (m_Vision.getTX() > setpoint) {
            m_Swerve.setControl(alignRequest.withVelocityY(VisionConstants.alignSpeed));
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_Swerve.setControl(idleRequest);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_Vision.isAlgaeAligned() || !m_Vision.getTV();
    }
}
