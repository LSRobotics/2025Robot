package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.claw.ClawConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem.WantedElevatorState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;

import java.lang.annotation.Retention;
import java.util.Map;

import javax.print.DocFlavor.INPUT_STREAM;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClawSubsystem extends SubsystemBase {
  private final ClawIO io;
  private final ClawIOInputsAutoLogged inputs = new ClawIOInputsAutoLogged();

  public enum WantedClawState{
    IDLE,
    MOVE_TO_L1,
    MOVE_TO_L2,
    MOVE_TO_L3,
    MOVE_TO_L4,
    MOVE_TO_ALGAE,
    MOVE_TO_PROCESSOR,
    MOVE_TO_BARGE,
    BARGE_SHOT,
    INTERMEDIATE_POS,
    MANUAL_CONTROL
  }

  public static Map<WantedClawState, Angle> stateToPosMap = Map.of(
      WantedClawState.IDLE,Radians.of( 0d),
      WantedClawState.MOVE_TO_L1, ClawConstants.L1ClawPosition,
      WantedClawState.MOVE_TO_L2, ClawConstants.L2ClawPosition,
      WantedClawState.MOVE_TO_L3, ClawConstants.L3ClawPosition,
      WantedClawState.MOVE_TO_L4, ClawConstants.L4ClawPosition,
      WantedClawState.MOVE_TO_ALGAE, ClawConstants.algaeClawPos,
      WantedClawState.MOVE_TO_PROCESSOR, ClawConstants.processorClawPos,
      WantedClawState.MOVE_TO_BARGE, ClawConstants.bargeClawPos,
      WantedClawState.INTERMEDIATE_POS, ClawConstants.intermediateClawPos,
      WantedClawState.BARGE_SHOT, ClawConstants.bargeClawPos);
  
  private enum ClawSystemState {
    IDLING,
    MOVING_TO_POSITION,
    HOLDING_POSITION,
    MANUAL
  }

  private ClawSystemState systemState = ClawSystemState.IDLING;
  private WantedClawState wantedState = WantedClawState.IDLE;
  private Angle targetPosition = Radian.of(0d);
  private double manualSpeed = 0d;

    private PIDController clawPID = new PIDController(
    ClawConstants.kP, 
    0, 
    ClawConstants.kD);
  private ArmFeedforward clawFeedforward = new ArmFeedforward(
    ClawConstants.kS, 
    ClawConstants.kG, 
    ClawConstants.kV, 
    ClawConstants.kA);

  public ClawSubsystem(ClawIO io) {
    this.io = io;

    clawPID.setTolerance(ClawConstants.tolerance.in(Radian));
  }

  public void setWantedState(WantedClawState state, double parameter) {
    this.wantedState = state;
    if (state==WantedClawState.MANUAL_CONTROL){
      this.manualSpeed = MathUtil.clamp(parameter, -1.0, 1.0);
      this.targetPosition = inputs.clawPosition;
    }
    else {
      this.targetPosition = stateToPosMap.getOrDefault(state, Radians.of(0d));
      this.manualSpeed = 0d;
    }
  }

  public boolean atTarget(){
    return Math.abs(inputs.clawPosition.in(Radian) - targetPosition.in(Radian)) < ClawConstants.tolerance.in(Radian);
  }

  private ClawSystemState handleStateTransition(){
    if (wantedState == WantedClawState.MANUAL_CONTROL){
      return ClawSystemState.MANUAL;
    }

    switch (systemState){
      case IDLING -> {
        if (wantedState != WantedClawState.IDLE) {
          return ClawSystemState.MOVING_TO_POSITION;
        }
      }
      case MOVING_TO_POSITION -> {
        if (atTarget()) {
          return ClawSystemState.HOLDING_POSITION;
        }
      }

      case HOLDING_POSITION -> {
        if (wantedState == WantedClawState.IDLE){
          return ClawSystemState.IDLING;
        }
      }
      case MANUAL -> {
        if (wantedState != WantedClawState.MANUAL_CONTROL) {
          return ClawSystemState.IDLING;
        }
      }

    }
    return systemState; // no state change
  }
  
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Claw", inputs);

    ClawSystemState nextState = handleStateTransition();
    if (nextState != systemState) {
      Logger.recordOutput("Claw/SystemState", nextState.toString());
      systemState = nextState;
    }
    Logger.recordOutput("Claw/WantedState", wantedState.toString());

    switch (systemState) {
      case IDLING -> {
        io.setClawVoltage(0.0);
      }
      case MOVING_TO_POSITION -> {
        clawPID.setSetpoint(targetPosition.in(Radian));

        double pidSpeed = clawPID.calculate(inputs.clawPosition.in(Radian));
        double speed = MathUtil.clamp(
          pidSpeed + clawFeedforward.calculate(inputs.clawPosition.in(Radian), 
          Math.signum(targetPosition.in(Radian)-inputs.clawPosition.in(Radian))*ClawConstants.feedforwardVelocity), 
          -ClawConstants.clampRangeforSpeed, ClawConstants.clampRangeforSpeed);

        Logger.recordOutput("Claw/PID+FF Output", speed);
        io.setClawMotorSpeed(speed);//This might be volatge
      }
      case HOLDING_POSITION -> {
        io.setClawVoltage(ClawConstants.kSVoltage);
      }

      case MANUAL -> {
        io.setClawMotorSpeed(manualSpeed);
        Logger.recordOutput("Claw/ManualSpeed", manualSpeed);
      }
    }

  }
}