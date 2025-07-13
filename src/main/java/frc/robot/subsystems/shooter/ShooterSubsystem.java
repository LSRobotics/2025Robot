// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;


public class ShooterSubsystem extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  
  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
  }
  
  public void runShooterMotor(double speed) {
    io.setSpeed(speed);
  }

  @Override
  public void periodic(){
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  
}
