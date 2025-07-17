package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/*
 * To add song:
 * 1. Get song MIDI
 * 2 Use this script to slice the MIDI into monophonic voices - https://github.com/slowpoke111/MidiFlattener
 * 3. In TunerX, go to tools->CHRP Generator and upload the MIDI from the previous step
 * 4. Copy the generated CHRP file to src/main/deploy/music, naming with camel case or snake case to ensure proper formatting
 */
public class playMusicCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final Orchestra m_Orchestra;
  private SendableChooser<String> songChooser;
  private String[] songPaths;
  private String[] songNames;

  // Helper to format song titles
  private static String formatSongTitle(String filename) {
    String name = filename;
    int dot = name.lastIndexOf('.');
    if (dot > 0) {
      name = name.substring(0, dot);
    }
    name = name.replace('_', ' ');

    name = name.replaceAll("([a-z])([A-Z])", "$1 $2");

    String[] words = name.split(" ");
    for (int i = 0; i < words.length; i++) {
      if (words[i].length() > 0) {
        words[i] = words[i].substring(0, 1).toUpperCase() + words[i].substring(1).toLowerCase();
      }
    }
    return String.join(" ", words);
  }

  {
    try {
      java.nio.file.Path musicDir = java.nio.file.Paths.get("src/main/deploy/music");
      songPaths = java.nio.file.Files.list(musicDir)
          .filter(java.nio.file.Files::isRegularFile)
          .map(path -> musicDir.relativize(path).toString().replace("\\", "/"))
          .toArray(String[]::new);

      songNames = java.nio.file.Files.list(musicDir)
          .filter(java.nio.file.Files::isRegularFile)
          .map(path -> formatSongTitle(path.getFileName().toString()))
          .toArray(String[]::new);
    } catch (java.io.IOException e) {
      e.printStackTrace();
      songPaths = new String[0];
      songNames = new String[0];
    }
  }
  
  public playMusicCommand() {
    m_Orchestra = new Orchestra();

    for (int i = 2; i < 10; i++) { //Swerve
      m_Orchestra.addInstrument(new TalonFX(i, "rio"));
    }

    m_Orchestra.addInstrument(new TalonFX(31, "rio")); //Claw

    
    songChooser = new SendableChooser<>();
    
    for (int i = 0; i < songPaths.length; i++) {
      if (i == 0) {
        songChooser.setDefaultOption(songNames[i], songPaths[i]);
      } else {
        songChooser.addOption(songNames[i], songPaths[i]);
      }
    }

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
