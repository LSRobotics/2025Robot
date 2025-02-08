// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ClawSubsystem;
import frc.robot.Constants.ClawConstants;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ClawToPositionCommand extends Command {
  private final ClawSubsystem m_claw;
  private double speed;
  private double previousPosition;
  private double currentPosition;
  private double currentVelocity;
  private double desiredPosition;
  
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ClawToPositionCommand(ClawSubsystem claw, double desiredPosition) {
    m_claw = claw;
    this.desiredPosition = desiredPosition;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      previousPosition = m_claw.getClawPosition();
      while(Math.abs(m_claw.getClawPosition() - desiredPosition) > ClawConstants.tolerance) {
        currentPosition = m_claw.getClawPosition();
        currentVelocity = currentPosition - previousPosition;
        speed = ClawConstants.kP * (desiredPosition - m_claw.getClawPosition()) - ClawConstants.kD * currentVelocity;
        previousPosition = currentPosition;
        if (speed > 0.1){
          speed = 0.1;
        }
        m_claw.runClawMotor(speed);
        System.out.println(speed);
        System.out.println("Difference" + (Math.abs(currentPosition - desiredPosition)));
      }
        m_claw.runClawMotor(0);
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_claw.runClawMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_claw.getClawPosition() == desiredPosition;
  }
}
