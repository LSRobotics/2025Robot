// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Configs.ElevatorConfigs;
import frc.robot.Constants.ElevatorConstants;
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
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ElevatorSubsystem extends SubsystemBase {
  public final SparkMax m_elevatorMotor1;
  public final SparkMax m_elevatorMotor2;
  public boolean isZeroed = false;

  public boolean fastModeBool = false;

  // public final SparkClosedLoopController m_elevatorFeedback;

  public DigitalInput elevatorLimit;
  private double currentTarget = 0;
  private RelativeEncoder elevatorEncoder;

  public ElevatorSubsystem() {
    m_elevatorMotor1 = new SparkMax(ElevatorConstants.kMotorID, MotorType.kBrushless);
    m_elevatorMotor2 = new SparkMax(ElevatorConstants.lMotorID, MotorType.kBrushless);

    // m_elevatorFeedback = m_elevatorMotor1.getClosedLoopController();

    elevatorEncoder = m_elevatorMotor1.getEncoder();

    elevatorLimit = new DigitalInput(0);

    m_elevatorMotor1.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_elevatorMotor2.configure(new SparkMaxConfig().follow(m_elevatorMotor1).idleMode(IdleMode.kBrake),
        ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatorEncoder.setPosition(0);
  }

  public void runElevatorMotorManual(double speed) {
    m_elevatorMotor1.set(speed);
  }

  public void setElevatorVoltage(double voltage) {
    m_elevatorMotor1.setVoltage(voltage);
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
    if (elevatorLimit.get() && !isZeroed) {
      elevatorEncoder.setPosition(0);
      isZeroed = true;
    } else if (!elevatorLimit.get()) {
      isZeroed = false;
    }
  }

  public double getPosition() {
    return m_elevatorMotor1.getEncoder().getPosition();
  }

  @Override

  public void periodic() {
    // m_elevatorFeedback.setReference(currentTarget,
    // ControlType.kMAXMotionPositionControl);
    // zeroEncoder();
    SmartDashboard.putNumber("Elevator Position: ", getPosition());
  }
}
