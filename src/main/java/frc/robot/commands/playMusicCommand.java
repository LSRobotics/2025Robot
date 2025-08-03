package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.File;

/*
 * To add song:
 * 1. Get song MIDI
 * 2 Use this script to slice the MIDI into monophonic voices - https://github.com/slowpoke111/MidiFlattener
 * 3. In TunerX, go to tools->CHRP Generator and upload the MIDI from the previous step
 * 4. Copy the generated CHRP file to src/main/deploy/music, naming with camel case or snake case to ensure proper formatting
 */
public class playMusicCommand extends Command {

  private final Orchestra m_Orchestra;
  private SendableChooser<String> songChooser;
  private String[] songPaths;
  private String[] songNames;

  private static final int[] MOTOR_CAN_IDS = {
    2, 3, 4, 5, 6, 7, 8, 9, // swerve
    31                      // Claw
  };

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
      // Use the correct deploy directory path for runtime
      File musicDir = new File(Filesystem.getDeployDirectory(), "music");
      
      if (musicDir.exists() && musicDir.isDirectory()) {
        File[] files = musicDir.listFiles((dir, name) -> name.toLowerCase().endsWith(".chrp"));
        
        if (files != null && files.length > 0) {
          songPaths = new String[files.length];
          songNames = new String[files.length];
          
          for (int i = 0; i < files.length; i++) {
            // Format paths as "music/filename.chrp"
            songPaths[i] = "music/" + files[i].getName();
            songNames[i] = formatSongTitle(files[i].getName());
          }
        } else {
          System.err.println("No .chrp files found in music directory");
          songPaths = new String[0];
          songNames = new String[0];
        }
      } else {
        System.err.println("Music directory not found: " + musicDir.getAbsolutePath());
        songPaths = new String[0];
        songNames = new String[0];
      }
    } catch (Exception e) {
      System.err.println("Error loading music files: " + e.getMessage());
      e.printStackTrace();
      songPaths = new String[0];
      songNames = new String[0];
    }
  }
  
  public playMusicCommand() {
    m_Orchestra = new Orchestra();

    for (int canId : MOTOR_CAN_IDS) {
      m_Orchestra.addInstrument(new TalonFX(canId, "rio"));
    }

    songChooser = new SendableChooser<>();
    
    if (songPaths.length > 0) {
      for (int i = 0; i < songPaths.length; i++) {
        if (i == 0) {
          songChooser.setDefaultOption(songNames[i], songPaths[i]);
        } else {
          songChooser.addOption(songNames[i], songPaths[i]);
        }
      }
    } else {
      // Add a placeholder if no songs are found
      songChooser.setDefaultOption("No songs found", "");
      SmartDashboard.putString("Song Error", "No music files found. Add .chrp files to deploy/music directory.");
    }

    SmartDashboard.putData("Song Selector", songChooser);
  }

  @Override
  public void initialize() { 
    String selectedSong = songChooser.getSelected(); 
    
    if (selectedSong != null && !selectedSong.isEmpty()) {
      StatusCode status = m_Orchestra.loadMusic(selectedSong); 
      if (status.isError()) {
        System.out.println("Error loading song: " + status.toString());
        SmartDashboard.putString("Song load status", "Error: " + status.toString());
      } else {
        SmartDashboard.putString("Song load status", "Loaded: " + selectedSong);
        m_Orchestra.play();
      }
    } else {
      SmartDashboard.putString("Song load status", "No song selected");
    }
  }

  @Override
  public void execute() {
    SmartDashboard.putString("Current Song", songChooser.getSelected());
    SmartDashboard.putBoolean("Is Playing", m_Orchestra.isPlaying());
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
