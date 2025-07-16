package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.Map;

public class ElevatorSubsystem extends SubsystemBase {
  public enum WantedElevatorState {
    IDLE,
    MOVE_TO_L1,
    MOVE_TO_L2,
    MOVE_TO_L3,
    MOVE_TO_L4,
    MOVE_TO_A1,
    MOVE_TO_A2,
    MOVE_TO_PROCESSOR,
    MOVE_TO_BARGE,
    MANUAL_CONTROL
  }

  public static final Map<WantedElevatorState, Double> stateToPosMap = Map.of(
      WantedElevatorState.IDLE, 0.0,
      WantedElevatorState.MOVE_TO_L1, ElevatorConstants.L1Height,
      WantedElevatorState.MOVE_TO_L2, ElevatorConstants.L2Height,
      WantedElevatorState.MOVE_TO_L3, ElevatorConstants.L3Height,
      WantedElevatorState.MOVE_TO_L4, ElevatorConstants.L4Height,
      WantedElevatorState.MOVE_TO_A1, ElevatorConstants.A1Height,
      WantedElevatorState.MOVE_TO_A2, ElevatorConstants.A2Height,
      WantedElevatorState.MOVE_TO_PROCESSOR, ElevatorConstants.processorHeight,
      WantedElevatorState.MOVE_TO_BARGE, ElevatorConstants.bargeHeight);

  private enum ElevatorSystemState {
    IDLING,
    MOVING_TO_POSITION,
    HOLDING_POSITION,
    MANUAL
  }

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private WantedElevatorState wantedState = WantedElevatorState.IDLE;
  private ElevatorSystemState systemState = ElevatorSystemState.IDLING;
  private double targetPosition = 0.0;
  private double manualSpeed = 0.0;

  private final Timer settleTimer = new Timer();
  
  private final PIDController m_ElevatorPID = new PIDController(
      ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);

  private final ElevatorFeedforward m_ElevatorFeedforward = new ElevatorFeedforward(
      ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV);

  public ElevatorSubsystem(ElevatorIO io) {
    this.io = io;
    m_ElevatorPID.setTolerance(ElevatorConstants.elevatorPosTolerance);
  }

  public void setWantedState(WantedElevatorState state, double parameter) {
    this.wantedState = state;
    if (state == WantedElevatorState.MANUAL_CONTROL) {
      this.manualSpeed = MathUtil.clamp(parameter, -1.0, 1.0); //speed
      this.targetPosition = inputs.position; // keep current pos as target
    } else {
      this.manualSpeed = 0.0; // reset when not in manual
      this.targetPosition = stateToPosMap.getOrDefault(state, 0.0);
    }
  }

  public boolean atTarget() {
    return Math.abs(inputs.position - targetPosition) <= ElevatorConstants.elevatorPosTolerance;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    ElevatorSystemState nextState = handleStateTransition();

    if (nextState != systemState) {
      Logger.recordOutput("Elevator/SystemState", nextState.toString());
      systemState = nextState;
    }
    Logger.recordOutput("Elevator/WantedState", wantedState.toString());

    switch (systemState) {
      case IDLING -> {
        io.setVoltage(0.0);
      }

      case MOVING_TO_POSITION -> {
        m_ElevatorPID.setSetpoint(targetPosition);
        double ff = m_ElevatorFeedforward.calculate(
            Math.signum(targetPosition - inputs.position) * ElevatorConstants.feedforwardVelocity);
        double pidOutput = m_ElevatorPID.calculate(inputs.position);
        double voltage = MathUtil.clamp(pidOutput + ff,
            -ElevatorConstants.clampRangeForSpeed, ElevatorConstants.clampRangeForSpeed);
        Logger.recordOutput("Elevator/Voltage PID Output", voltage);
        io.setVoltage(voltage);
      }

      case HOLDING_POSITION -> {
        io.setVoltage(ElevatorConstants.elevatorHoldVoltage);
      }

      case MANUAL -> {
        io.setSpeed(manualSpeed);
        Logger.recordOutput("Elevator/ManualSpeed", manualSpeed);
      }
    }
  }

  private ElevatorSystemState handleStateTransition() {
    if (wantedState == WantedElevatorState.MANUAL_CONTROL) {
      return ElevatorSystemState.MANUAL;
    }

    switch (systemState) {
      case IDLING -> {
        if (wantedState != WantedElevatorState.IDLE) {
          return ElevatorSystemState.MOVING_TO_POSITION;
        }
      }

      case MOVING_TO_POSITION -> {
        if (atTarget()) {
          settleTimer.reset();
          settleTimer.start();
          return ElevatorSystemState.HOLDING_POSITION;
        }
      }

      case HOLDING_POSITION -> {
        if (wantedState == WantedElevatorState.IDLE) {
          return ElevatorSystemState.IDLING;
        } else if (!atTarget()) {
          return ElevatorSystemState.MOVING_TO_POSITION;
        }
      }

      case MANUAL -> {
        if (wantedState != WantedElevatorState.MANUAL_CONTROL) {
          return ElevatorSystemState.IDLING;
        }
      }
    }

    return systemState;
  }
}
