// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.
// package frc.robot.commands;

// import edu.wpi.first.math.MathUtil;
// import frc.robot.subsystems.claw.ClawSubsystem;
// import frc.robot.subsystems.claw.ClawConstants;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import static edu.wpi.first.units.Units.Radian;
// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ArmFeedforward;

// /** An example command that uses an example subsystem. */
// public class ClawToPositionCommand extends Command {
//   private final ClawSubsystem m_claw;
//   private PIDController clawPID = new PIDController(
//     ClawConstants.kP, 
//     0, 
//     ClawConstants.kD);
//   private ArmFeedforward clawFeedforward = new ArmFeedforward(
//     ClawConstants.kS, 
//     ClawConstants.kG, 
//     ClawConstants.kV, 
//     ClawConstants.kA);

//     double targetPos;
  
//   /**
//    * Creates a new ExampleCommand.
//    *
//    * @param subsystem The subsystem used by this command.
//    */
//   public ClawToPositionCommand(ClawSubsystem claw, Angle desiredPosition) {
//     m_claw = claw;

//     clawPID.setTolerance(ClawConstants.tolerance.in(Radian));
//     clawPID.setSetpoint(desiredPosition.in(Radian));

//     targetPos = desiredPosition.in(Radian);
    
//     addRequirements(claw);
//   }

//   @Override
//   public void initialize() {}

//   @Override
//   public void execute() {
//     double pidSpeed = clawPID.calculate(m_claw.getClawPosition().in(Radian));
//     double speed = MathUtil.clamp(
//       pidSpeed + clawFeedforward.calculate(m_claw.getClawPosition().in(Radian), 
//       Math.signum(targetPos-m_claw.getClawPosition().in(Radian))*ClawConstants.feedforwardVelocity), 
//       -ClawConstants.clampRangeforSpeed, ClawConstants.clampRangeforSpeed);

//     m_claw.runClawMotor(speed);

//     SmartDashboard.putNumber("Angle", m_claw.getClawPosition().in(Radian));
//     SmartDashboard.putNumber("Feedforward/back Speed", speed);
//   }

//   @Override
//   public void end(boolean interrupted) {
//     m_claw.runClawMotor(0); // find a way to fight gravity TODO
//   }

//   @Override
//   public boolean isFinished() {
//     return clawPID.atSetpoint();
//   }
// }
