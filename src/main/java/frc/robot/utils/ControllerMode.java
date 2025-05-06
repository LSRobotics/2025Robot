package frc.robot.utils;
import java.util.Map;

public enum ControllerMode {
    SCORING_1,
    DRIVING_2,
    OTHER_3;
    
    public static final Map<ControllerMode, String> ModeToColor = Map.of(
        SCORING_1, "#ff0000",
        DRIVING_2, "#00ff00",
        OTHER_3, "#0000ff"
    );
}
