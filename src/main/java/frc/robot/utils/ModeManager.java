package frc.robot.utils;

public class ModeManager {
    private static ControllerMode currentMode = ControllerMode.SCORING;

    public static ControllerMode getMode() {
        return currentMode;
    }

    public static void setMode(ControllerMode newMode) {
        currentMode = newMode;
    }

    public static void cycleNext() {
        ControllerMode[] modes = ControllerMode.values();
        int nextIndex = (currentMode.ordinal() + 1) % modes.length;
        currentMode = modes[nextIndex];
    }

    public static void cyclePrev() {
        ControllerMode[] modes = ControllerMode.values();
        int prevIndex = (currentMode.ordinal() - 1 + modes.length) % modes.length;
        currentMode = modes[prevIndex];
    }
}
