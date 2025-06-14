package frc.robot.utils;
import java.util.Map;

public enum ControllerMode {
    NoGamePieceState,
    CORALState,
    ALGAEState;
    
    public static final Map<ControllerMode, String> ModeToColor = Map.of(
        NoGamePieceState, "#ff0000",
        CORALState, "#00ff00",
        ALGAEState, "#0000ff"
    );
}
