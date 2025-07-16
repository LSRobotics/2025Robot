package frc.robot;

import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.WantedElevatorState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {

    private final ElevatorSubsystem elevatorSubsystem;

    public enum SuperstructureState {
        IDLE,
        SCORE_L1,
        SCORE_L2,
        SCORE_L3,
        SCORE_L4,
        SCORE_A1,
        SCORE_A2,
        SCORE_PROCESSOR,
        SCORE_BARGE,
        INTAKE,
        MANUAL_ELEVATOR,
        MANUAL_CLAW
    }

    private SuperstructureState currentState = SuperstructureState.IDLE;

    public Superstructure(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
    }


    public void setSuperstructureState(SuperstructureState state, double parameter) {
        currentState = state;
        switch (state) {
            case IDLE:
                elevatorSubsystem.setWantedState(WantedElevatorState.IDLE, 0.0);
                break;
            case SCORE_L1:
                elevatorSubsystem.setWantedState(WantedElevatorState.MOVE_TO_L1, 0.0);
                break;
            case SCORE_L2:
                elevatorSubsystem.setWantedState(WantedElevatorState.MOVE_TO_L2, 0.0);
                break;
            case SCORE_L3:
                elevatorSubsystem.setWantedState(WantedElevatorState.MOVE_TO_L3, 0.0);
                break;
            case SCORE_L4:
                elevatorSubsystem.setWantedState(WantedElevatorState.MOVE_TO_L4, 0.0);
                break;
            case MANUAL_ELEVATOR:
                // parameter is manual speed here
                elevatorSubsystem.setWantedState(WantedElevatorState.MANUAL_CONTROL, parameter);
                break;
            case SCORE_A1:
                elevatorSubsystem.setWantedState(WantedElevatorState.MOVE_TO_A1, 0.0);

            case SCORE_A2:
                elevatorSubsystem.setWantedState(WantedElevatorState.MOVE_TO_A2, 0.0);
                break;

            case SCORE_PROCESSOR:
                elevatorSubsystem.setWantedState(WantedElevatorState.MOVE_TO_PROCESSOR, 0.0);
                break;

            case INTAKE:
                break;
            default:
                // For safety, default to idle
                elevatorSubsystem.setWantedState(WantedElevatorState.IDLE, 0.0);
                break;
        }
    }


    public void setSuperstructureState(SuperstructureState state) {
        setSuperstructureState(state, 0.0);
    }

    @Override
    public void periodic() {

    }
    public boolean isElevatorAtTarget() {
        return elevatorSubsystem.atTarget();
    }

    public SuperstructureState getCurrentState() {
        return currentState;
    }
}
