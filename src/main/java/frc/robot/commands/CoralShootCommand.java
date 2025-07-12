// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class CoralShootCommand extends Command {
  private final ShooterSubsystem m_shooter;
  private final double speed;
  
 
  public CoralShootCommand(ShooterSubsystem shooter, double speed) {
    m_shooter = shooter;
    this.speed = speed;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
      m_shooter.runShooterMotor(speed);
    }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_shooter.runShooterMotor(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}