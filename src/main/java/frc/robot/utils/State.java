package frc.robot.utils;

public interface State {
    
    void enter(StateMachine stateMachine);

    void exit(StateMachine stateMachine);

    State update(StateMachine stateMachine);
}
