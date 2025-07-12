// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {
  private final SparkFlex shooterMotor;
  
  public ShooterSubsystem() {
    shooterMotor = new SparkFlex(Constants.ShooterConstants.shooterMotorID, MotorType.kBrushless);
  }
  
  public void runShooterMotor(double speed) {
    shooterMotor.set(speed);
  }

  
}
