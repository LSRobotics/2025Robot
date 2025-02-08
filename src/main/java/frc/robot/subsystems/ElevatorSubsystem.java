package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  private final TalonFX elevatorMotor;
  private final PIDController heightController;
  private final DutyCycleOut encoder;

  public ElevatorSubsystem() {
    elevatorMotor = new TalonFX(Constants.ElevatorConstants.kMotorID);
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

    elevatorMotor.setControl(encoder.withOutput(output));
  }

  // should be correct afaik
  public double calculateMotorDist() {
    return elevatorMotor.getPosition().getValue().magnitude() * 
    Constants.ElevatorConstants.kMotorCircumference;
  }
}