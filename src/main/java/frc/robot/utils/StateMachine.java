package frc.robot.utils;

public class StateMachine {
    
    private State currentState;
    private State previousState;
    

    public StateMachine() {
        this.currentState = null;
        this.previousState = null;
    }
    

    public StateMachine(State initialState) {
        this.currentState = null;
        this.previousState = null;
        
        if (initialState != null) {
            changeState(initialState);
        }
    }
    

    public State getCurrentState() {
        return currentState;
    }
    

    public State getPreviousState() {
        return previousState;
    }
    

    public void changeState(State newState) {
        if (currentState != null) {
            currentState.exit(this);
        }
        
        previousState = currentState;
        
        currentState = newState;
        currentState.enter(this);
    }
    

    public boolean update() {
        if (currentState == null) {
            return false;
        }
        
        State nextState = currentState.update(this);
        
        if (nextState != null && nextState != currentState) {
            changeState(nextState);
        }
        
        return true;
    }
    

    public void revertToPreviousState() {
        if (previousState != null) {
            changeState(previousState);
        }
    }
}
