package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ModeManager {
    private static ControllerMode currentMode = ControllerMode.CORALState;

    public static ControllerMode getMode() {
        return currentMode;
    }

    public static String getModeColor() {
        return ControllerMode.ModeToColor.get(currentMode);
    }

    public static void setMode(ControllerMode newMode) {
        SmartDashboard.putString("Controller Mode", currentMode.toString());
        currentMode = newMode;
    }

    public static void cycleNext() {
        ControllerMode[] modes = ControllerMode.values();
        int nextIndex = (currentMode.ordinal() + 1) % modes.length;
        currentMode = modes[nextIndex];
        SmartDashboard.putString("Controller Mode", currentMode.toString());
        System.out.println(nextIndex);
    }

    public static void cyclePrev() {
        ControllerMode[] modes = ControllerMode.values();
        int prevIndex = (currentMode.ordinal() - 1 + modes.length) % modes.length;
        currentMode = modes[prevIndex];
        SmartDashboard.putString("Controller Mode", currentMode.toString());
    }
}
