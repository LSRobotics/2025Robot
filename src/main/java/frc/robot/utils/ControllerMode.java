package frc.robot.utils;
import java.util.Map;

public enum ControllerMode {
    SCORING,
    DRIVING,
    OTHER;
    
    public static final Map<ControllerMode, String> ModeToColor = Map.of(
        SCORING, "#ff0000",
        DRIVING, "#00ff00",
        OTHER, "#0000ff"
    );
}
