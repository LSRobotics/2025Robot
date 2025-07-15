// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.LegacyConstants.MultiversalConstants;

import java.util.Timer;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.PathPlannerAuto;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  Timer endOfMatchTimer = new Timer();

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {

    Logger.recordMetadata("ProjectName", GitBuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", GitBuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", GitBuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", GitBuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", GitBuildConstants.GIT_BRANCH);
    Logger.recordMetadata("WPILIB Version", GitBuildConstants.VERSION);
    Logger.recordMetadata("Is Real", "" + Robot.isReal());
    Logger.recordMetadata("Uncommited Changes", "" + GitBuildConstants.DIRTY);

    Logger.recordMetadata("Java Version", SystemBuildConstants.JAVA_VERSION);
    Logger.recordMetadata("Gradle Version", SystemBuildConstants.GRADLE_VERSION);
    Logger.recordMetadata("OS Name", SystemBuildConstants.OS_NAME);

    // Log each vendorlib version
    // for (String vendorlib : SystemBuildConstants.VENDORLIBS) {
    // String[] parts = vendorlib.split(":", 2);
    // if (parts.length == 2) {
    // Logger.recordMetadata("VendorLib: " + parts[0], parts[1]);
    // }
    // }

    Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    Logger.start();

    OtherLoggers.start(this, new PowerDistribution(1, PowerDistribution.ModuleType.kRev));
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  public void RobotInit() {
    // Sets the LED color to blue when the robot turns on
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    SignalLogger.stop();
    endOfMatchTimer.cancel();
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = new PathPlannerAuto(m_robotContainer.getAutonomousCommand())
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    // m_robotContainer.isTeleop = () -> false;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    SignalLogger.start();

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // m_robotContainer.isTeleop = () -> true;

    // Sets the LED color to gold after match ends
    /*
     * endOfMatchTimer.schedule(new TimerTask() {
     * 
     * @Override
     * public void run() {
     * m_led.LEDColor(LEDConstants.twinklesColorOneAndTwo);
     * }
     * }, new Date().getTime() + LEDConstants.matchTimeInMilliseconds);
     */
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
