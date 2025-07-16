// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.drive.SwerveConstants.SwerveSpeedConsts;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.claw.ClawConstants;
import frc.robot.subsystems.claw.ClawIOTalonFX;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIOSparkMax;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.AutonAdjustCommand;
import frc.robot.commands.AutonAlgaeAlignCommand;
// import frc.robot.commands.MannualElevatorCommand;
// import frc.robot.commands.ManualClawCommand;
import frc.robot.commands.CoralShootCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.CommandSwerveDrivetrainLogger;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.leds.LEDIOBlinken;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;


import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIOSparkFlex;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;


import edu.wpi.first.net.PortForwarder;
// import frc.robot.commands.ClawToPositionCommand;
import frc.robot.commands.CoralIntakeCommand;
// import frc.robot.commands.ElevatorToPositionCommand;
import java.util.function.BooleanSupplier;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //The robot's subsystems and commands are defined here...
  private final ClawSubsystem m_claw = new ClawSubsystem(new ClawIOTalonFX());
  private final ShooterSubsystem m_shooter = new ShooterSubsystem(new ShooterIOSparkFlex());
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem(new ElevatorIOSparkMax());



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

    // private DoubleSupplier swerveSpeed = () -> m_elevator.getSwerveSpeed();
    
 /*  public BooleanSupplier isTeleop = () -> false;
  public BooleanSupplier coralPresent = this::coralPresent;
  public BooleanSupplier canRunIntake = () -> isTeleop.getAsBoolean() && coralPresent.getAsBoolean();
  Trigger runIndexerTrigger = new Trigger(canRunIntake); 
  */
  Trigger runIndexerTrigger = new Trigger(this::coralPresent);
  Trigger manualClawTriggerUp = new Trigger(() -> yOperator.getAsDouble() > 0);
  Trigger manualClawTriggerDown = new Trigger(() -> yOperator.getAsDouble() < 0);
  // Trigger elevatorL1 = new Trigger(()->Math.abs(m_elevator.getPosition()-ElevatorConstants.L1Height) < ElevatorConstants.elevatorPrecision);


  // private final SendableChooser<Command> autoChooser;

  public final LEDSubsystem m_LEDs = new LEDSubsystem(new LEDIOBlinken());
  public final VisionSubsystem m_Vision = new VisionSubsystem(m_LEDs);
  
  private final TimeOfFlight m_rangeSensor = new TimeOfFlight(ClawConstants.sensorID);
//   private final BooleanSupplier intermediatePos = () -> Math.abs((m_claw.getClawPosition().minus(ClawConstants.intermediateClawPos)).in(Radian))<ClawConstants.tolerance.in(Radian);
//   public final Trigger clawAtIntermediate = new Trigger(intermediatePos);

   public RobotContainer() {
      for (int port = 5800; port <= 5809; port++) {
          PortForwarder.add(port, "limelight.local", port);
      }

      CommandSwerveDrivetrainLogger.start("CommandSwerveDrivetrain", m_drivetrain, 0.025);

      //NamedCommands.registerCommand("Align", 
      //  new AlignCommand(m_drivetrain, m_Vision, true).withTimeout(2));

      // NamedCommands.registerCommand("Shoot", 
      //   new CoralShootCommand(m_shooter, ShooterConstants.slowShooterSpeed).withTimeout(1));
      
      // NamedCommands.registerCommand("Intake", 
      //   new WaitUntilCommand(this::coralPresent).andThen(
      //   new CoralIntakeCommand(m_shooter, m_elevator, ShooterConstants.intakeSpeed, this::coralPresent)).andThen(
      //   new WaitCommand(0.3)
      //   ));

      // NamedCommands.registerCommand("L1", 
      //   new ClawToPositionCommand(m_claw, ClawConstants.intermediateClawPos).andThen(Commands.parallel(
      //   new ElevatorToPositionCommand(m_elevator,ElevatorConstants.L1Height), 
      //   new WaitUntilCommand(() -> (Math.abs(m_elevator.getPosition()-ElevatorConstants.L1Height) < ElevatorConstants.elevatorPrecision)).andThen(
      //   new ClawToPositionCommand(m_claw, ClawConstants.L1ClawPosition)))
      //   ));
      // NamedCommands.registerCommand("L2", 
      //   new ClawToPositionCommand(m_claw, ClawConstants.intermediateClawPos).andThen(Commands.parallel(
      //   new ElevatorToPositionCommand(m_elevator,ElevatorConstants.L2Height), 
      //   new WaitUntilCommand(() -> (Math.abs(m_elevator.getPosition()-ElevatorConstants.L2Height) < ElevatorConstants.elevatorPrecision)).andThen(
      //   new ClawToPositionCommand(m_claw, ClawConstants.L2ClawPosition)))
      //   ));
      // NamedCommands.registerCommand("L3", 
      //   new ClawToPositionCommand(m_claw, ClawConstants.intermediateClawPos).andThen(Commands.parallel(
      //   new ElevatorToPositionCommand(m_elevator,ElevatorConstants.L3Height), 
      //   new WaitUntilCommand(() -> (Math.abs(m_elevator.getPosition()-ElevatorConstants.L3Height) < ElevatorConstants.elevatorPrecision)).andThen(
      //   new ClawToPositionCommand(m_claw, ClawConstants.L3ClawPosition)))
      //   ));
      // NamedCommands.registerCommand("L4", 
      //   new ClawToPositionCommand(m_claw, ClawConstants.intermediateClawPos).andThen(Commands.parallel(
      //   new ElevatorToPositionCommand(m_elevator,ElevatorConstants.L4Height), 
      //   new WaitUntilCommand(() -> (Math.abs(m_elevator.getPosition()-ElevatorConstants.L4Height) < ElevatorConstants.elevatorPrecision)).andThen(
      //   new ClawToPositionCommand(m_claw, ClawConstants.AutonL4Claw)))
      //   ));
      // NamedCommands.registerCommand("A1", 
      //   new ClawToPositionCommand(m_claw, ClawConstants.intermediateClawPos).andThen(Commands.parallel(
      //   new ElevatorToPositionCommand(m_elevator,ElevatorConstants.A1Height), 
      //   new WaitUntilCommand(() -> (Math.abs(m_elevator.getPosition()-ElevatorConstants.A1Height) < ElevatorConstants.elevatorPrecision)).andThen(new ClawToPositionCommand(m_claw, ClawConstants.algaeClawPos)).andThen(
      //   new InstantCommand(() -> m_shooter.runShooterMotor(ShooterConstants.algaeSpeed))
      //   ))
      // ));
      // NamedCommands.registerCommand("A2", 
      //   new ClawToPositionCommand(m_claw, ClawConstants.intermediateClawPos).andThen(Commands.parallel(
      //   new ElevatorToPositionCommand(m_elevator,ElevatorConstants.A2Height), 
      //   new WaitUntilCommand(() -> (Math.abs(m_elevator.getPosition()-ElevatorConstants.A2Height) < ElevatorConstants.elevatorPrecision)).andThen(new ClawToPositionCommand(m_claw, ClawConstants.algaeClawPos)).andThen(
      //   new InstantCommand(() -> m_shooter.runShooterMotor(ShooterConstants.algaeSpeed))
      //   ))
      // ));
      // NamedCommands.registerCommand("Processor",         
      //   new ClawToPositionCommand(m_claw, ClawConstants.processorClawPos).andThen(Commands.parallel(
      //   new ElevatorToPositionCommand(m_elevator,ElevatorConstants.processorHeight), 
      //   new WaitUntilCommand(() -> (Math.abs(m_elevator.getPosition()-ElevatorConstants.processorHeight) < ElevatorConstants.elevatorPrecision)).andThen(
      //   new ClawToPositionCommand(m_claw, ClawConstants.processorClawPos)).andThen(
      //   new InstantCommand(() -> m_shooter.runShooterMotor(ShooterConstants.algaeSpeed))
      //   ))
      // ));

      // NamedCommands.registerCommand("BargePos", 
      //   new ClawToPositionCommand(m_claw, ClawConstants.algaeClawPos).andThen(Commands.parallel(
      //   new ElevatorToPositionCommand(m_elevator,ElevatorConstants.bargeHeight), 
      //   new WaitUntilCommand(() -> (Math.abs(m_elevator.getPosition()-ElevatorConstants.bargeHeight) < ElevatorConstants.elevatorPrecision)).andThen(
      //   new ClawToPositionCommand(m_claw, ClawConstants.bargeClawPos)).andThen(
      //   ))
      // ));

      // NamedCommands.registerCommand("BargeShot", Commands.parallel(
      //   new CoralShootCommand(m_shooter, ShooterConstants.bargeAlgaeShooterSpeed), 
      //   new ManualClawCommand(m_claw, -ClawConstants.bargeClawSpeed)).withTimeout(0.25));

      // NamedCommands.registerCommand("ElevatorDown", 
      //   new MannualElevatorCommand(m_elevator, -0.03).withTimeout(1));

      // NamedCommands.registerCommand("Align", new AutonAdjustCommand(m_Vision, m_drivetrain, true));
      // NamedCommands.registerCommand("Algae Align", new AutonAlgaeAlignCommand(m_Vision, m_drivetrain));


      //  m_rangeSensor.setRangingMode(RangingMode.Short, 24);
      //  autoChooser = AutoBuilder.buildAutoChooser("Tests");
      //  SmartDashboard.putData("Auto Mode", autoChooser);
 
       configureBindings();
   }
   public Command getAutonomousCommand() {
     // An example command will be run in autonomous
    //  return autoChooser.getSelected();
    return new InstantCommand();
   }

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
    //.onTrue(new ExampleCommand(m_exampleSubsystem));
    m_drivetrain.registerTelemetry(new Telemetry(MaxSpeed)::telemeterize); // experiment with this telemetry TODO
    // m_drivetrain.setDefaultCommand(
    //     // Drivetrain will execute this command periodically
    //     m_drivetrain.applyRequest(() ->
    //         drive.withVelocityX(-m_driverController.getLeftY() * swerveSpeed.getAsDouble()) // Drive forward with negative Y (forward)
    //              .withVelocityY(-m_driverController.getLeftX() *  swerveSpeed.getAsDouble()) // Drive left with negative X (left)
    //              .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
    //     ));

    // m_driverController.a().whileTrue(m_drivetrain.applyRequest(() -> brake));
    //m_driverController.b().whileTrue(m_drivetrain.applyRequest(() ->
    //    point.withModuleDirection(new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX()))
    //));

    //m_driverController.x().whileTrue(new AlignCommand(m_drivetrain, m_Vision,true));

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

    double diagonalSpeed = SwerveSpeedConsts.slowSpeed / Math.sqrt(2);

    m_driverController.pov(45).whileTrue(m_drivetrain.applyRequest(() ->
        forwardStraight.withVelocityX(diagonalSpeed).withVelocityY(-diagonalSpeed))
    );

    m_driverController.pov(135).whileTrue(m_drivetrain.applyRequest(() ->
        forwardStraight.withVelocityX(-diagonalSpeed).withVelocityY(-diagonalSpeed))
    );

    m_driverController.pov(225).whileTrue(m_drivetrain.applyRequest(() ->
        forwardStraight.withVelocityX(-diagonalSpeed).withVelocityY(diagonalSpeed))
    );

    m_driverController.pov(315).whileTrue(m_drivetrain.applyRequest(() ->
        forwardStraight.withVelocityX(diagonalSpeed).withVelocityY(diagonalSpeed))
    );

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    // m_driverController.start().onTrue(Commands.runOnce(SignalLogger::start));
    // m_driverController.back().onTrue(Commands.runOnce(SignalLogger::stop));
    // m_driverController.rightBumper().and(m_driverController.y()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kForward));
    // m_driverController.rightBumper().and(m_driverController.x()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kReverse));
    // m_driverController.leftBumper().and(m_driverController.y()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kForward));
    // m_driverController.leftBumper().and(m_driverController.x()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    //m_driverController.leftBumper().onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldCentric()));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

    // DoubleSupplier leftY = () -> m_operatorController.getLeftY();
    // DoubleSupplier elevatorPosition = () -> m_elevator.getPosition();

    // BooleanSupplier elevatorAtL1 = () -> (Math.abs(m_elevator.getPosition()-ElevatorConstants.L1Height) < ElevatorConstants.elevatorPrecision);
    // BooleanSupplier elevatorAtL2 = () -> (Math.abs(m_elevator.getPosition()-ElevatorConstants.L2Height) < ElevatorConstants.elevatorPrecision);
    // BooleanSupplier elevatorAtL3 = () -> (Math.abs(m_elevator.getPosition()-ElevatorConstants.L3Height) < ElevatorConstants.elevatorPrecision);
    // BooleanSupplier elevatorAtL4 = () -> (Math.abs(m_elevator.getPosition()-ElevatorConstants.L4Height) < ElevatorConstants.elevatorPrecision);

    // BooleanSupplier elevatorAtA1 = () -> (Math.abs(m_elevator.getPosition()-ElevatorConstants.A1Height) < ElevatorConstants.elevatorPrecision);
    // BooleanSupplier elevatorAtA2 = () -> (Math.abs(m_elevator.getPosition()-ElevatorConstants.A2Height) < ElevatorConstants.elevatorPrecision);

    // BooleanSupplier elevatorAtProcessor = () -> (Math.abs(m_elevator.getPosition()-ElevatorConstants.processorHeight) < ElevatorConstants.elevatorPrecision);
    // BooleanSupplier elevatorAtBarge = () -> (Math.abs(m_elevator.getPosition()-ElevatorConstants.bargeHeight) < ElevatorConstants.elevatorPrecision);

    // Trigger MannualElevatorUp = new Trigger(() -> leftY.getAsDouble() < -0.8);
    // Trigger MannualElevatorDown = new Trigger(() -> leftY.getAsDouble() > 0.8);

    // // CLAW AND ELEVATOR POSITION CONTROLS
    // m_operatorController.a().onTrue(new ConditionalCommand(new ClawToPositionCommand(m_claw, ClawConstants.L1ClawPosition), 
    //   new ClawToPositionCommand(m_claw, ClawConstants.intermediateClawPos).andThen(Commands.parallel(
    //   new ElevatorToPositionCommand(m_elevator,ElevatorConstants.L1Height), 
    //   new WaitUntilCommand(() -> (Math.abs(m_elevator.getPosition()-ElevatorConstants.L1Height) < ElevatorConstants.elevatorPrecision)).andThen(
    //   new ClawToPositionCommand(m_claw, ClawConstants.L1ClawPosition)))
    //   ), elevatorAtL1));

    // m_operatorController.b().onTrue(new ConditionalCommand(new ClawToPositionCommand(m_claw, ClawConstants.L2ClawPosition),
    //   new ClawToPositionCommand(m_claw, ClawConstants.intermediateClawPos).andThen(Commands.parallel(
    //   new ElevatorToPositionCommand(m_elevator,ElevatorConstants.L2Height), 
    //   new WaitUntilCommand(() -> (Math.abs(m_elevator.getPosition()-ElevatorConstants.L2Height) < ElevatorConstants.elevatorPrecision)).andThen(
    //   new ClawToPositionCommand(m_claw, ClawConstants.L2ClawPosition)))
    //   ), elevatorAtL2));

    // m_operatorController.b().onTrue(new ConditionalCommand(new ClawToPositionCommand(m_claw, ClawConstants.L2ClawPosition),
    //   new ClawToPositionCommand(m_claw, ClawConstants.intermediateClawPos).andThen(Commands.parallel(
    //   new ElevatorToPositionCommand(m_elevator,ElevatorConstants.L2Height), 
    //   new WaitUntilCommand(() -> (Math.abs(m_elevator.getPosition()-ElevatorConstants.L2Height) < ElevatorConstants.elevatorPrecision)).andThen(
    //   new ClawToPositionCommand(m_claw, ClawConstants.L2ClawPosition)))
    // ), elevatorAtL2));
    
    // m_operatorController.x().onTrue(new ConditionalCommand(new ClawToPositionCommand(m_claw, ClawConstants.L3ClawPosition),
    //   new ClawToPositionCommand(m_claw, ClawConstants.intermediateClawPos).andThen(Commands.parallel(
    //   new ElevatorToPositionCommand(m_elevator,ElevatorConstants.L3Height), 
    //   new WaitUntilCommand(() -> (Math.abs(m_elevator.getPosition()-ElevatorConstants.L3Height) < ElevatorConstants.elevatorPrecision)).andThen(
    //   new ClawToPositionCommand(m_claw, ClawConstants.L3ClawPosition)))
    //   ), elevatorAtL3));
    
    // m_operatorController.y().onTrue(new ConditionalCommand(new ClawToPositionCommand(m_claw, ClawConstants.L4ClawPosition),
    //   new ClawToPositionCommand(m_claw, ClawConstants.intermediateClawPos).andThen(Commands.parallel(
    //   new ElevatorToPositionCommand(m_elevator,ElevatorConstants.L4Height), 
    //   new WaitUntilCommand(() -> (Math.abs(m_elevator.getPosition()-ElevatorConstants.L4Height) < ElevatorConstants.elevatorPrecision)).andThen(
    //   new ClawToPositionCommand(m_claw, ClawConstants.L4ClawPosition)))
    //   ), elevatorAtL4));

    // m_operatorController.povUp().onTrue(
    //   new ConditionalCommand(new ClawToPositionCommand(m_claw, ClawConstants.algaeClawPos), 
    //   new ClawToPositionCommand(m_claw, ClawConstants.intermediateClawPos).andThen(Commands.parallel(
    //   new ElevatorToPositionCommand(m_elevator,ElevatorConstants.A2Height), 
    //   new WaitUntilCommand(() -> (Math.abs(m_elevator.getPosition()-ElevatorConstants.A2Height) < ElevatorConstants.elevatorPrecision))).andThen(
    //   new ClawToPositionCommand(m_claw, ClawConstants.algaeClawPos)
    //   )), elevatorAtA2));

    // m_operatorController.povDown().onTrue(
    //   new ConditionalCommand(new ClawToPositionCommand(m_claw, ClawConstants.algaeClawPos), 
    //   new ClawToPositionCommand(m_claw, ClawConstants.intermediateClawPos).andThen(Commands.parallel(
    //   new ElevatorToPositionCommand(m_elevator,ElevatorConstants.A1Height), 
    //   new WaitUntilCommand(() -> (Math.abs(m_elevator.getPosition()-ElevatorConstants.A1Height) < ElevatorConstants.elevatorPrecision)).andThen(
    //   new ClawToPositionCommand(m_claw, ClawConstants.algaeClawPos))
    //   )), elevatorAtA1));

    // m_driverController.leftBumper().onTrue(      
    //   new ConditionalCommand(new ClawToPositionCommand(m_claw, ClawConstants.processorClawPos), 
    //   new ClawToPositionCommand(m_claw, ClawConstants.processorClawPos).andThen(Commands.parallel(
    //   new ElevatorToPositionCommand(m_elevator,ElevatorConstants.processorHeight), 
    //   new WaitUntilCommand(() -> (Math.abs(m_elevator.getPosition()-ElevatorConstants.processorHeight) < ElevatorConstants.elevatorPrecision)).andThen(
    //   new ClawToPositionCommand(m_claw, ClawConstants.processorClawPos))
    //   )), elevatorAtProcessor));

    // m_operatorController.leftBumper().onTrue(      
    //   new ConditionalCommand(new ClawToPositionCommand(m_claw, ClawConstants.algaeClawPos), 
    //   new ClawToPositionCommand(m_claw, ClawConstants.algaeClawPos).andThen(Commands.parallel(
    //   new ElevatorToPositionCommand(m_elevator,ElevatorConstants.bargeHeight), 
    //   new WaitUntilCommand(() -> (Math.abs(m_elevator.getPosition()-ElevatorConstants.bargeHeight) < ElevatorConstants.elevatorPrecision)).andThen(
    //   new ClawToPositionCommand(m_claw, ClawConstants.bargeClawPos))
    //   )), elevatorAtBarge));
  
    


    // zero the claw angle . . . MAKE SURE TO DO THIS BEFORE DISABLING THE BOT OR GOING INTO A MATCH
    // m_operatorController.rightBumper().onTrue(new ClawToPositionCommand(m_claw, 0));

    // failsafe for manual control
    // manualClawTriggerUp.whileTrue(new ManualClawCommand(m_claw, -ClawConstants.manualClawSpeed));
    // manualClawTriggerDown.whileTrue(new ManualClawCommand(m_claw, ClawConstants.manualClawSpeed));
    // MannualElevatorUp.whileTrue(new MannualElevatorCommand(m_elevator, 0.15));
    // MannualElevatorDown.whileTrue(new MannualElevatorCommand(m_elevator, -0.03));

    // SHOOTER AND INTAKE CONTROLS
    m_operatorController.leftTrigger().whileTrue(new CoralShootCommand(m_shooter, -ShooterConstants.slowShooterSpeed));
    runIndexerTrigger.whileTrue(new CoralIntakeCommand(m_shooter, m_elevator, ShooterConstants.intakeSpeed, ()->coralPresent()));
    m_operatorController.rightTrigger().whileTrue(new CoralShootCommand(m_shooter, ShooterConstants.slowShooterSpeed));
    m_driverController.rightBumper().whileTrue(new CoralShootCommand(m_shooter, ShooterConstants.L1ShooterSpeed));


    // BRAKE CLAW USING VOLTAGE
    // m_operatorController.povRight().toggleOnTrue(new InstantCommand(() -> m_claw.motorVoltage(ClawConstants.kSVoltage)));
    // m_operatorController.povLeft().toggleOnTrue(new InstantCommand(() -> m_claw.motorVoltage(0)));

    //Algae Controls
    m_driverController.leftTrigger().whileTrue(new AlgaeIntakeCommand(m_shooter));
    m_driverController.rightTrigger().whileTrue(new CoralShootCommand(m_shooter, ShooterConstants.slowShooterSpeed));
    // m_operatorController.rightBumper().onTrue(Commands.parallel(
    //   new ManualClawCommand(m_claw, -ClawConstants.bargeClawSpeed), 
    //   new CoralShootCommand(m_shooter, ShooterConstants.bargeAlgaeShooterSpeed)).withTimeout(0.25));

    //m_driverController.a().toggleOnTrue(new InstantCommand(() -> m_elevator.setElevatorVoltage(1.25)));
    
    // m_driverController.b().whileTrue(
    //   new ConditionalCommand(
    //     new ConditionalCommand(
    //       new MannualElevatorCommand(m_elevator, 0),
    //       new MannualElevatorCommand(m_elevator, -0.5), elevatorAtL1),
    //     new ClawToPositionCommand(m_claw, ClawConstants.intermediateClawPos), clawAtIntermediate));

    //  m_driverController.b().whileTrue(
    //     new ConditionalCommand(
    //       new MannualElevatorCommand(m_elevator, -0.1), 
    //       new MannualElevatorCommand(m_elevator, -0.95), elevatorL1)); 

    m_driverController.x().whileTrue(new AutonAdjustCommand(m_Vision, m_drivetrain, true));
   // m_driverController.a().whileTrue(new CoralShootCommand(m_shooter, ShooterConstants.slowShooterSpeed));
   // m_driverController.y().whileTrue(new CoralShootCommand(m_shooter, -ShooterConstants.slowShooterSpeed));

  //  m_driverController.a().and(m_driverController.y()).onTrue(new InstantCommand(() -> m_elevator.fastModeBool=!m_elevator.fastModeBool));
   //m_driverController.a().whileTrue(new AutonAlgaeAlignCommand(m_Vision, m_drivetrain));



      
  }

  public boolean coralPresent() {
    BooleanSupplier coralPresent = () -> (m_rangeSensor.getRange() > 1) && (m_rangeSensor.getRange() < 170);
    SmartDashboard.putNumber("Dist", m_rangeSensor.getRange());
    boolean isPresent = coralPresent.getAsBoolean();
    return isPresent;
  }

}