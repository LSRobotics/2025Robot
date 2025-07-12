

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ElevatorToPositionCommand extends Command {
  private final ElevatorSubsystem m_Elevator;

  private final PIDController m_ElevatorPID = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
  private final ElevatorFeedforward m_ElevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV);

  private final double targetPos;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorToPositionCommand(ElevatorSubsystem elevator, double pos) {
    m_Elevator = elevator;
    
    m_ElevatorPID.setSetpoint(pos);
    m_ElevatorPID.setTolerance(ElevatorConstants.elevatorPosTolerance);
    
    targetPos = pos;

    addRequirements(elevator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
  
    double voltage = MathUtil.clamp(
      m_ElevatorPID.calculate(m_Elevator.getPosition())
     +m_ElevatorFeedforward.calculate(Math.signum(targetPos-m_Elevator.getPosition())*ElevatorConstants.feedforwardVelocity),
    -ElevatorConstants.clampRangeForSpeed, ElevatorConstants.clampRangeForSpeed);
    System.out.println(targetPos-m_Elevator.getPosition());
    SmartDashboard.putNumber("Elevator Voltage PID",voltage);
    SmartDashboard.putNumber("Elevator Position", m_Elevator.getPosition());
    m_Elevator.setElevatorVoltage(voltage);
  }




  @Override
  public void end(boolean interrupted) {
    m_Elevator.setElevatorVoltage(ElevatorConstants.elevatorHoldVoltage);
  }

  @Override
  public boolean isFinished() {
    return m_ElevatorPID.atSetpoint();
    //return false;
  }
}