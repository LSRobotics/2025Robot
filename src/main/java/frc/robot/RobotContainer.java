// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;
import java.util.jar.Attributes.Name;

import frc.robot.commands.AlignSlide;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.SwerveSpeedConsts;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.MannualElevatorCommand;
import frc.robot.commands.ManualClawCommand;
import frc.robot.commands.RunShooterCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.ShooterConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import frc.robot.commands.ClawToPositionCommand;
import frc.robot.commands.ElevatorToPositionCommand;
import java.util.function.BooleanSupplier;
import frc.robot.subsystems.ClawSubsystem;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Telemetry;

import edu.wpi.first.math.filter.SlewRateLimiter;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ClawSubsystem m_claw = new ClawSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();


  private DoubleSupplier swerveSpeed = () -> m_elevator.getSwerveSpeed();
  private final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  DoubleSupplier yOperator = () -> m_operatorController.getRightY();
  Trigger runIndexerTrigger = new Trigger(this::coralPresent);
  Trigger 
  manualClawTriggerUp = new Trigger(() -> yOperator.getAsDouble() > 0);
  Trigger manualClawTriggerDown = new Trigger(() -> yOperator.getAsDouble() < 0);

  private final SendableChooser<Command> autoChooser;
  
    private final VisionSubsystem m_Vision = new VisionSubsystem();

  private final TimeOfFlight m_rangeSensor = new TimeOfFlight(ClawConstants.sensorID);

   public RobotContainer() {
      for (int port = 5800; port <= 5809; port++) {
          PortForwarder.add(port, "limelight.local", port);
      }

      NamedCommands.registerCommand("Align", new AlignCommand(m_drivetrain, m_Vision).withTimeout(2));
      NamedCommands.registerCommand("Shoot", new RunShooterCommand(m_shooter, ShooterConstants.slowShooterSpeed).withTimeout(1));

      NamedCommands.registerCommand("L1", 
        new ClawToPositionCommand(m_claw, ClawConstants.intermediateClawPos).andThen(Commands.parallel(
        new ElevatorToPositionCommand(m_elevator,ElevatorConstants.L1Height), 
        new WaitUntilCommand(() -> (Math.abs(m_elevator.getPosition()-ElevatorConstants.L1Height) < ElevatorConstants.elevatorPrecision)).andThen(new ClawToPositionCommand(m_claw, ClawConstants.L1ClawPosition)))
        ));
      NamedCommands.registerCommand("L2", 
        new ClawToPositionCommand(m_claw, ClawConstants.intermediateClawPos).andThen(Commands.parallel(
        new ElevatorToPositionCommand(m_elevator,ElevatorConstants.L2Height), 
        new WaitUntilCommand(() -> (Math.abs(m_elevator.getPosition()-ElevatorConstants.L2Height) < ElevatorConstants.elevatorPrecision)).andThen(new ClawToPositionCommand(m_claw, ClawConstants.L2ClawPosition)))
        ));
      NamedCommands.registerCommand("L3", 
        new ClawToPositionCommand(m_claw, ClawConstants.intermediateClawPos).andThen(Commands.parallel(
        new ElevatorToPositionCommand(m_elevator,ElevatorConstants.L3Height), 
        new WaitUntilCommand(() -> (Math.abs(m_elevator.getPosition()-ElevatorConstants.L3Height) < ElevatorConstants.elevatorPrecision)).andThen(new ClawToPositionCommand(m_claw, ClawConstants.L3ClawPosition)))
        ));
      NamedCommands.registerCommand("L4", 
        new ClawToPositionCommand(m_claw, ClawConstants.intermediateClawPos).andThen(Commands.parallel(
        new ElevatorToPositionCommand(m_elevator,ElevatorConstants.L4Height), 
        new WaitUntilCommand(() -> (Math.abs(m_elevator.getPosition()-ElevatorConstants.L4Height) < ElevatorConstants.elevatorPrecision)).andThen(new ClawToPositionCommand(m_claw, ClawConstants.L4ClawPosition)))
        ));
      NamedCommands.registerCommand("A1", 
        new ClawToPositionCommand(m_claw, ClawConstants.intermediateClawPos).andThen(Commands.parallel(
        new ElevatorToPositionCommand(m_elevator,ElevatorConstants.A1Height), 
        new WaitUntilCommand(() -> (Math.abs(m_elevator.getPosition()-ElevatorConstants.A1Height) < ElevatorConstants.elevatorPrecision)).andThen(new ClawToPositionCommand(m_claw, ClawConstants.algaeClawPos)).andThen(
        new InstantCommand(() -> m_shooter.runShooterMotor(ShooterConstants.algaeSpeed))
        ))
      ));
      NamedCommands.registerCommand("A2", 
        new ClawToPositionCommand(m_claw, ClawConstants.intermediateClawPos).andThen(Commands.parallel(
        new ElevatorToPositionCommand(m_elevator,ElevatorConstants.A2Height), 
        new WaitUntilCommand(() -> (Math.abs(m_elevator.getPosition()-ElevatorConstants.A2Height) < ElevatorConstants.elevatorPrecision)).andThen(new ClawToPositionCommand(m_claw, ClawConstants.algaeClawPos)).andThen(
        new InstantCommand(() -> m_shooter.runShooterMotor(ShooterConstants.algaeSpeed))
        ))
      ));

       m_rangeSensor.setRangingMode(RangingMode.Short, 24);
       autoChooser = AutoBuilder.buildAutoChooser("Tests");
       SmartDashboard.putData("Auto Mode", autoChooser);
 
       configureBindings();
   }
   public Command getAutonomousCommand() {
     // An example command will be run in autonomous
     return autoChooser.getSelected();
   }

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
    //.onTrue(new ExampleCommand(m_exampleSubsystem));
    m_drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        m_drivetrain.applyRequest(() ->
            drive.withVelocityX(-m_driverController.getLeftY() * swerveSpeed.getAsDouble()) // Drive forward with negative Y (forward)
                 .withVelocityY(-m_driverController.getLeftX() *  swerveSpeed.getAsDouble()) // Drive left with negative X (left)
                 .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    // m_driverController.a().whileTrue(m_drivetrain.applyRequest(() -> brake));
    m_driverController.b().whileTrue(m_drivetrain.applyRequest(() ->
        point.withModuleDirection(new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX()))
    ));

    m_driverController.x().whileTrue(new AlignCommand(m_drivetrain, m_Vision));
    m_driverController.a().onTrue(new AlignSlide(m_drivetrain, 1, 1, 0.5));

    m_driverController.pov(0).whileTrue(m_drivetrain.applyRequest(() ->
        forwardStraight.withVelocityX(SwerveSpeedConsts.slowSpeed).withVelocityY(0))
    );
    m_driverController.pov(180).whileTrue(m_drivetrain.applyRequest(() ->
        forwardStraight.withVelocityX(-SwerveSpeedConsts.slowSpeed).withVelocityY(0))
    );

    m_driverController.pov(90).whileTrue(m_drivetrain.applyRequest(() ->
        forwardStraight.withVelocityX(0).withVelocityY(-SwerveSpeedConsts.slowSpeed))
    );

    m_driverController.pov(270).whileTrue(m_drivetrain.applyRequest(() ->
        forwardStraight.withVelocityX(0).withVelocityY(SwerveSpeedConsts.slowSpeed))
    );    

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    m_driverController.back().and(m_driverController.y()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kForward));
    m_driverController.back().and(m_driverController.x()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kReverse));
    m_driverController.start().and(m_driverController.y()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kForward));
    m_driverController.start().and(m_driverController.x()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    m_driverController.leftBumper().onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldCentric()));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

    DoubleSupplier leftY = () -> m_operatorController.getLeftY();
    DoubleSupplier elevatorPosition = () -> m_elevator.getPosition();

    BooleanSupplier elevatorAtL1 = () -> (Math.abs(m_elevator.getPosition()-ElevatorConstants.L1Height) < ElevatorConstants.elevatorPrecision);
    BooleanSupplier elevatorAtL2 = () -> (Math.abs(m_elevator.getPosition()-ElevatorConstants.L2Height) < ElevatorConstants.elevatorPrecision);
    BooleanSupplier elevatorAtL3 = () -> (Math.abs(m_elevator.getPosition()-ElevatorConstants.L3Height) < ElevatorConstants.elevatorPrecision);
    BooleanSupplier elevatorAtL4 = () -> (Math.abs(m_elevator.getPosition()-ElevatorConstants.L4Height) < ElevatorConstants.elevatorPrecision);

    BooleanSupplier elevatorAtA1 = () -> (Math.abs(m_elevator.getPosition()-ElevatorConstants.A1Height) < ElevatorConstants.elevatorPrecision);
    BooleanSupplier elevatorAtA2 = () -> (Math.abs(m_elevator.getPosition()-ElevatorConstants.A2Height) < ElevatorConstants.elevatorPrecision);

    Trigger MannualElevatorUp = new Trigger(() -> leftY.getAsDouble() < -0.8);
    Trigger MannualElevatorDown = new Trigger(() -> leftY.getAsDouble() > 0.8);

    // CLAW AND ELEVATOR POSITION CONTROLS
    m_operatorController.a().onTrue(new ConditionalCommand(new ClawToPositionCommand(m_claw, ClawConstants.L1ClawPosition), 
      new ClawToPositionCommand(m_claw, ClawConstants.intermediateClawPos).andThen(Commands.parallel(
      new ElevatorToPositionCommand(m_elevator,ElevatorConstants.L1Height), 
      new WaitUntilCommand(() -> (Math.abs(m_elevator.getPosition()-ElevatorConstants.L1Height) < ElevatorConstants.elevatorPrecision)).andThen(
      new ClawToPositionCommand(m_claw, ClawConstants.L1ClawPosition)))
      ), elevatorAtL1));

    m_operatorController.b().onTrue(new ConditionalCommand(new ClawToPositionCommand(m_claw, ClawConstants.L2ClawPosition),
      new ClawToPositionCommand(m_claw, ClawConstants.intermediateClawPos).andThen(Commands.parallel(
      new ElevatorToPositionCommand(m_elevator,ElevatorConstants.L2Height), 
      new WaitUntilCommand(() -> (Math.abs(m_elevator.getPosition()-ElevatorConstants.L2Height) < ElevatorConstants.elevatorPrecision)).andThen(
      new ClawToPositionCommand(m_claw, ClawConstants.L2ClawPosition)))
      ), elevatorAtL2));
    
    m_operatorController.x().onTrue(new ConditionalCommand(new ClawToPositionCommand(m_claw, ClawConstants.L3ClawPosition),
      new ClawToPositionCommand(m_claw, ClawConstants.intermediateClawPos).andThen(Commands.parallel(
      new ElevatorToPositionCommand(m_elevator,ElevatorConstants.L3Height), 
      new WaitUntilCommand(() -> (Math.abs(m_elevator.getPosition()-ElevatorConstants.L3Height) < ElevatorConstants.elevatorPrecision)).andThen(
      new ClawToPositionCommand(m_claw, ClawConstants.L3ClawPosition)))
      ), elevatorAtL3));
    
    m_operatorController.y().onTrue(new ConditionalCommand(new ClawToPositionCommand(m_claw, ClawConstants.L4ClawPosition),
      new ClawToPositionCommand(m_claw, ClawConstants.intermediateClawPos).andThen(Commands.parallel(
      new ElevatorToPositionCommand(m_elevator,ElevatorConstants.L4Height), 
      new WaitUntilCommand(() -> (Math.abs(m_elevator.getPosition()-ElevatorConstants.L4Height) < ElevatorConstants.elevatorPrecision)).andThen(
      new ClawToPositionCommand(m_claw, ClawConstants.L4ClawPosition)))
      ), elevatorAtL4));

    m_operatorController.povUp().onTrue(
      new ConditionalCommand(new ClawToPositionCommand(m_claw, ClawConstants.algaeClawPos), 
      new ClawToPositionCommand(m_claw, ClawConstants.intermediateClawPos).andThen(Commands.parallel(
      new ElevatorToPositionCommand(m_elevator,ElevatorConstants.A2Height), 
      new WaitUntilCommand(() -> (Math.abs(m_elevator.getPosition()-ElevatorConstants.A2Height) < ElevatorConstants.elevatorPrecision))).andThen(
      new ClawToPositionCommand(m_claw, ClawConstants.algaeClawPos)).andThen(
      new InstantCommand(() -> m_shooter.runShooterMotor(ShooterConstants.algaeSpeed))
      )), elevatorAtA2));

    m_operatorController.povDown().onTrue(
      new ConditionalCommand(new ClawToPositionCommand(m_claw, ClawConstants.algaeClawPos), 
      new ClawToPositionCommand(m_claw, ClawConstants.intermediateClawPos).andThen(Commands.parallel(
      new ElevatorToPositionCommand(m_elevator,ElevatorConstants.A1Height), 
      new WaitUntilCommand(() -> (Math.abs(m_elevator.getPosition()-ElevatorConstants.A1Height) < ElevatorConstants.elevatorPrecision)).andThen(
      new ClawToPositionCommand(m_claw, ClawConstants.algaeClawPos))).andThen(
        new InstantCommand(() -> m_shooter.runShooterMotor(ShooterConstants.algaeSpeed))
    )), elevatorAtA1));


    // zero the claw angle . . . MAKE SURE TO DO THIS BEFORE DISABLING THE BOT OR GOING INTO A MATCH
    // m_operatorController.rightBumper().onTrue(new ClawToPositionCommand(m_claw, 0));

    // failsafe for manual control
    manualClawTriggerUp.whileTrue(new ManualClawCommand(m_claw, -ClawConstants.manualClawSpeed));
    manualClawTriggerDown.whileTrue(new ManualClawCommand(m_claw, ClawConstants.manualClawSpeed));
    MannualElevatorUp.whileTrue(new MannualElevatorCommand(m_elevator, 0.15));
    MannualElevatorDown.whileTrue(new MannualElevatorCommand(m_elevator, -0.03));

    // SHOOTER AND INTAKE CONTROLS
    m_operatorController.leftTrigger().whileTrue(new RunShooterCommand(m_shooter, -ShooterConstants.slowShooterSpeed));
    runIndexerTrigger.whileTrue(new RunShooterCommand(m_shooter, ShooterConstants.intakeSpeed));
    m_operatorController.rightTrigger().whileTrue(new RunShooterCommand(m_shooter, ShooterConstants.slowShooterSpeed));

    // BRAKE CLAW USING VOLTAGE
    m_operatorController.povRight().toggleOnTrue(new InstantCommand(() -> m_claw.motorVoltage(ClawConstants.kSVoltage)));
    m_operatorController.povLeft().toggleOnTrue(new InstantCommand(() -> m_claw.motorVoltage(0)));

    //Algae Controls
    m_driverController.leftTrigger().whileTrue(new RunShooterCommand(m_shooter, -ShooterConstants.algaeSpeed));
    m_driverController.rightTrigger().whileTrue(new RunShooterCommand(m_shooter, ShooterConstants.slowShooterSpeed));

  }

  public boolean coralPresent() {
    BooleanSupplier coralPresent = () -> (m_rangeSensor.getRange() > 1) && (m_rangeSensor.getRange() < 170);
    SmartDashboard.putNumber("Dist", m_rangeSensor.getRange());
    boolean isPresent = coralPresent.getAsBoolean();
    return isPresent;
  }


}