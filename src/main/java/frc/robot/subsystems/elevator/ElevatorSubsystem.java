// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Configs.ElevatorConfigs;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.Constants.SwerveSpeedConsts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ElevatorSubsystem extends SubsystemBase {

  public boolean isZeroed = false;

  public boolean fastModeBool = false;

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();


  private double currentTarget = 0;

  public ElevatorSubsystem(ElevatorIO io) {
    this.io = io;
  }

  public void runElevatorMotorManual(double speed) {
    io.setSpeed(speed);
  }

  public void setElevatorVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public void setPosition(double position) {
    currentTarget = position;
  }

  public double getSwerveSpeed() {
    SmartDashboard.putBoolean("Fast mode bool", fastModeBool);

    if (getPosition() < (ElevatorConstants.L1Height+5) && fastModeBool) {
      SmartDashboard.putNumber("Swerve Speed", 4.0);
      return 4.0;
    } 
    else {
      fastModeBool = false;
      SmartDashboard.putNumber("Swerve Speed", SwerveSpeedConsts.bargeSpeed +
      (Math.pow(((ElevatorConstants.L4Height - getPosition()) / ElevatorConstants.bargeHeight), 2)
          * (SwerveSpeedConsts.L1Speed - SwerveSpeedConsts.bargeSpeed)));
      return SwerveSpeedConsts.bargeSpeed +
          (Math.pow(((ElevatorConstants.L4Height - getPosition()) / ElevatorConstants.bargeHeight), 2)
              * (SwerveSpeedConsts.L1Speed - SwerveSpeedConsts.bargeSpeed));
    }
  }

  public void zeroEncoder() {
    if (io.getElevatorLimitSwitch() && !isZeroed) {
      io.zeroEncoder();
      isZeroed = true;
    } else if (!io.getElevatorLimitSwitch()) {
      isZeroed = false;
    }
  }

  public double getPosition() {
    return inputs.position;
  }

  @Override
  public void periodic() {
    // m_elevatorFeedback.setReference(currentTarget,
    // ControlType.kMAXMotionPositionControl);
    // zeroEncoder();
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    Logger.recordOutput("Elevator/SwerveSpeed", getSwerveSpeed());
  }
}
