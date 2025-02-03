// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;

import javax.net.ssl.SSLEngineResult.Status;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class playMusicCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

   private final Orchestra m_Orchestra;
   private int counter = 0;

   private final String[] songPaths = {"music/AlmaMater.chrp","music/piratesOfCaribbean.chrp"};
  public playMusicCommand() {
    m_Orchestra = new Orchestra();
    for (int i=2;i<10;i++){
    m_Orchestra.addInstrument(new TalonFX(i, "rio"));
    }

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    StatusCode status = m_Orchestra.loadMusic(songPaths[counter]);
    if (status.isError()){
      System.out.println(status.toString());
    }
    m_Orchestra.play();
    counter=(counter+1)%songPaths.length;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Orchestra.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
