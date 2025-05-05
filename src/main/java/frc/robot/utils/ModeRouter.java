package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.Map;

public class ModeRouter extends Command {
    private final Map<ControllerMode, Command> modeCommands;

    public ModeRouter(Map<ControllerMode, Command> modeCommands) {
        this.modeCommands = modeCommands;
    }

    @Override
    public void initialize() {
        Command command = modeCommands.get(ModeManager.getMode());
        if (command != null) {
            command.schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
