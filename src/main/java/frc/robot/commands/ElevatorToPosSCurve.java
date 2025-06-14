package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants.ElevatorConstants;

import frc.robot.utils.SCurveMotionProfile;
import frc.robot.utils.SCurveMotionProfile.State;

public class ElevatorToPosSCurve extends Command {
    private final ElevatorSubsystem m_Elevator;
    private final PIDController m_PID;
    private final ElevatorFeedforward m_FF;
    private final SCurveMotionProfile m_Profile;
    private final double targetPos;

    private double startT;

    public ElevatorToPosSCurve(ElevatorSubsystem elevator, double targetPosition) {
        m_Elevator = elevator;
        targetPos = targetPosition;

        m_PID = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
        m_PID.setTolerance(ElevatorConstants.elevatorPosTolerance);

        m_FF = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV,
                ElevatorConstants.kA);

        double currentPos = m_Elevator.getPosition();
        double currentVel = 0; // add method to subsytem latter

        double maxVelocity = 3;
        double maxAcceleration = 2;
        double maxJerk = 1.5; 

        m_Profile = new SCurveMotionProfile(
                currentPos, currentVel,
                targetPos, 0,
                maxVelocity, maxAcceleration, maxJerk);

        addRequirements(m_Elevator);
    }

    public ElevatorToPosSCurve(ElevatorSubsystem elevator, double targetPosition, double maxVelocity,
            double maxAcceleration, double maxJerk) {
        m_Elevator = elevator;
        targetPos = targetPosition;

        m_PID = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
        m_PID.setTolerance(ElevatorConstants.elevatorPosTolerance);

        m_FF = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV,
                ElevatorConstants.kA);

        double currentPos = m_Elevator.getPosition();
        double currentVel = 0; // add method to subsytem latter

        m_Profile = new SCurveMotionProfile(
                currentPos, currentVel,
                targetPos, 0,
                maxVelocity, maxAcceleration, maxJerk);

        addRequirements(m_Elevator);
    }

    @Override
    public void initialize() {
        startT = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        double elapsedTime = Timer.getFPGATimestamp() - startT;

        double dt = 0.02; // 50 hz from command schduler

        State currentState = m_Profile.sample(elapsedTime);
        State nextState = m_Profile.sample(elapsedTime + dt);

        m_PID.setSetpoint(currentState.position);

        double pidOutput = m_PID.calculate(m_Elevator.getPosition());

        double feedforwardVoltage = m_FF.calculateWithVelocities(currentState.velocity, nextState.velocity);

        double voltage = pidOutput + feedforwardVoltage;

        voltage = MathUtil.clamp(voltage, -ElevatorConstants.clampRangeForSpeed, ElevatorConstants.clampRangeForSpeed);

        m_Elevator.setElevatorVoltage(voltage);

        // SmartDashboard.putNumber("SCurve Target Pos", currentState.position);
        // SmartDashboard.putNumber("SCurve Target Vel", currentState.velocity);
        // SmartDashboard.putNumber("SCurve Target Accel", currentState.acceleration);
        // SmartDashboard.putNumber("Elevator Voltage", voltage);

    }

    @Override
    public void end(boolean interrupted) {
        m_Elevator.setElevatorVoltage(ElevatorConstants.elevatorHoldVoltage);
    }

    @Override
    public boolean isFinished() {
        double elapsedTime = Timer.getFPGATimestamp() - startT;

        return elapsedTime >= m_Profile.getTotalTime() && m_PID.atSetpoint();
    }
}
