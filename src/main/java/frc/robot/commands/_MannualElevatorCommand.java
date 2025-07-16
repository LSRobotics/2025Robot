// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.elevator.ElevatorConstants;
// import frc.robot.subsystems.elevator.ElevatorSubsystem;

// public class MannualElevatorCommand extends Command{
//     private final ElevatorSubsystem elevator;
//     private final double speed; 

//     public MannualElevatorCommand(ElevatorSubsystem elevatorSubsystem, double speed){
//         elevator = elevatorSubsystem;
//         this.speed = speed;
//     }

//     @Override
//     public void initialize(){
//         elevator.runElevatorMotorManual(speed);
//     }

//     @Override
//     public void end(boolean interrupted){
//         elevator.runElevatorMotorManual(ElevatorConstants.elevatorManualHoldVoltage);
//     }

//     @Override
//     public boolean isFinished(){
//         return false;
//     }
// }
