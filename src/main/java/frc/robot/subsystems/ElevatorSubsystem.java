package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  private final SparkMax elevatorMotor;
  private final PIDController heightController;
  private final DutyCycleOut encoder;

  public ElevatorSubsystem() {
    elevatorMotor = new SparkMax(Constants.ElevatorConstants.kMotorID, MotorType.kBrushless);
    heightController = new PIDController(
        Constants.ElevatorConstants.kP,
        Constants.ElevatorConstants.kI,
        Constants.ElevatorConstants.kD);
    heightController.setTolerance(Constants.ElevatorConstants.kTolerance);
    encoder = new DutyCycleOut(0);
  }

  public void setHeight(double targetHeight) {
    heightController.setSetpoint(targetHeight);
  }
  public void changeHeight(double deltaHeight) {
    heightController.setSetpoint(heightController.getSetpoint() + deltaHeight);
  }

  @Override
  public void periodic() {
    double currentHeight = calculateMotorDist();
    double output = heightController.calculate(currentHeight);

    elevatorMotor.set(output);
  }

  // should be correct afaik
  public double calculateMotorDist() {
    return elevatorMotor.get() * 
    Constants.ElevatorConstants.kMotorCircumference;
  }
}