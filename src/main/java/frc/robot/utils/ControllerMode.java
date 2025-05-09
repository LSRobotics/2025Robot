package frc.robot.utils;
import java.util.Map;

public enum ControllerMode {
    DRIVER,
    OPERATOR,
    OTHER;
    
    public static final Map<ControllerMode, String> ModeToColor = Map.of(
        DRIVER, "#ff0000",
        OPERATOR, "#00ff00",
        OTHER, "#0000ff"
    );
}
