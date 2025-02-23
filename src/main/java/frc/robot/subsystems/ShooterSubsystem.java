// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX shooterMotor = new TalonFX(ShooterConstants.shooterMotorID);
  
  public ShooterSubsystem() {
    shooterMotor.setNeutralMode(NeutralModeValue.Brake);
  }
  
  public void runShooterMotor(double speed) {
    shooterMotor.set(speed);
  }

  
}
