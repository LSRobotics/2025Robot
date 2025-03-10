// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;

public class CoralIntakeCommand extends Command {
  private final ShooterSubsystem m_shooter;
  private final double speed;
  private final BooleanSupplier coralPresent;
 
  public CoralIntakeCommand(ShooterSubsystem shooter, double speed, BooleanSupplier coralPresent) {
    m_shooter = shooter;
    this.speed = speed;
    this.coralPresent = coralPresent;
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
    System.out.println("Coral Intake Command Ended");
    
  }

  @Override
  public boolean isFinished() {
    return !coralPresent.getAsBoolean();
  }
}