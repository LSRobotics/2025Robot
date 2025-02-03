package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class playMusicCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final Orchestra m_Orchestra;
  private SendableChooser<String> songChooser;
  private String[] songPaths = {"music/AlmaMater.chrp", "music/piratesOfCaribbean.chrp"};
  
  public playMusicCommand() {
    m_Orchestra = new Orchestra();

    for (int i = 2; i < 10; i++) {
      m_Orchestra.addInstrument(new TalonFX(i, "rio"));
    }
    
    songChooser = new SendableChooser<>();
    
    songChooser.addOption("Alma Mater (2 Parts)", songPaths[0]);
    songChooser.addOption("Pirates of the Caribbean (6 parts)", songPaths[1]);

    SmartDashboard.putData("Song Selector", songChooser);
  }

  @Override
  public void initialize() {
    String selectedSong = songChooser.getSelected();

    StatusCode status = m_Orchestra.loadMusic(selectedSong);
    if (status.isError()) {
      System.out.println(status.toString());
    }

    m_Orchestra.play();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    m_Orchestra.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
