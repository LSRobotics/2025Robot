package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.*;

import frc.robot.utils.ControllerMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.Map;

//Example use from robotcontainer
//private final Bindings bindings = new Bindings(m_shooter, m_climber, m_drive); Include all subsytems
//m_operatorController.x().onTrue(new ModeRouter(bindings.xButtonCommands));


public class Bindings {

    public final Map<ControllerMode, Command> xButtonCommands;

    public Bindings(ClawSubsystem m_Claw, CommandSwerveDrivetrain m_Swerve, ElevatorSubsystem m_Elevator, LEDSubsystem m_LEDs, ShooterSubsystem m_Shooter, VisionSubsystem m_Vision) {
        xButtonCommands = Map.ofEntries(
            Map.entry(ControllerMode.SCORING_1, new CoralShootCommand(m_Shooter, ShooterConstants.slowShooterSpeed)),
            Map.entry(ControllerMode.DRIVING_2, new InstantCommand(() -> m_Elevator.fastModeBool=!m_Elevator.fastModeBool))
        );
    }
}
