package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.LimelightHelpers.*;
import frc.robot.subsystems.leds.LEDSubsystem;

public class VisionSubsystem extends SubsystemBase {
  private RawFiducial[] fiducials;
  public final LEDSubsystem m_LEDs;

  public VisionSubsystem(LEDSubsystem leds) {
    m_LEDs = leds;

    config();
  }

  public static class NoSuchTargetException extends RuntimeException { // No fiducial fonund
    public NoSuchTargetException(String message) {
      super(message);
    }
  }

  public void config() {
    // Enable if fps is an issue
    // LimelightHelpers.setCropWindow("", -0.5, 0.5, -0.5, 0.5);

    LimelightHelpers.setCameraPose_RobotSpace( // maybe put in consts.java
        "",
        0,
        0,
        0.3048,
        0,
        0,
        0);
    LimelightHelpers.SetFiducialIDFiltersOverride("", new int[] {3, 6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22 });

    //SmartDashboard.putNumber("Rotate P", 0.0);
    //SmartDashboard.putNumber("Rotate D", 0.0);

  }

  @Override
  public void periodic() {
    fiducials = LimelightHelpers.getRawFiducials("");
    SmartDashboard.putBoolean("Is aligned:", isAligned());
    if (isAligned()) {
      m_LEDs.runLEDs(0.77);
    } else if (isAlgaeAligned()) {
      m_LEDs.runLEDs(0.93);
    } else {
      m_LEDs.runLEDs(0.85);
    }
  }

  public RawFiducial getClosestFiducial() {
    if (fiducials == null || fiducials.length == 0) {
      throw new NoSuchTargetException("No fiducials found.");
    }

    RawFiducial closest = fiducials[0];
    double minDistance = closest.ta;
    // Linear search for close
    for (RawFiducial fiducial : fiducials) {
      if (fiducial.ta > minDistance) {
        closest = fiducial;
        minDistance = fiducial.ta;
      }
    }
    return closest;
  }

  public RawFiducial getClosestFiducial(boolean throwErr) {
    if (fiducials == null || fiducials.length == 0) {
      return null;
    }

    RawFiducial closest = fiducials[0];
    double minDistance = closest.ta;
    // Linear search for close
    for (RawFiducial fiducial : fiducials) {
      if (fiducial.ta > minDistance) {
        closest = fiducial;
        minDistance = fiducial.ta;
      }
    }
    return closest;
  }

  // Linear searcgh by id
  public RawFiducial getFiducialWithId(int id) {

    for (RawFiducial fiducial : fiducials) {
      if (fiducial.id == id) {
        return fiducial;
      }
    }
    throw new NoSuchTargetException("Can't find ID: " + id);
  }

  public RawFiducial getFiducialWithId(int id, boolean verbose) {// Debug
    StringBuilder availableIds = new StringBuilder();

    for (RawFiducial fiducial : fiducials) {
      if (availableIds.length() > 0) {
        availableIds.append(", ");
      } // Error reporting
      availableIds.append(fiducial.id);

      if (fiducial.id == id) {
        return fiducial;
      }
    }
    throw new NoSuchTargetException("Cannot find: " + id + ". IN view:: " + availableIds.toString());
  }
  //Withing 2.5 deg of 21 or -21 deg and less than 0.75 m away
  public boolean isAligned() {
    try {
      if (Math.abs(getTX() - VisionConstants.branchAngle) < VisionConstants.branchTolerance
          && getClosestFiducial().distToRobot < 0.75) {
        return true;
      } // Left
      if (Math.abs(getTX() + VisionConstants.branchAngle) < VisionConstants.branchTolerance
          && getClosestFiducial().distToRobot < 0.75) {
        return true;
      } // Right
    } catch (Exception e) {
      return false;
    }
    return false;
  }
  //Withing 2.5 deg of 0 deg and less than 0.75 m away
  public boolean isAlgaeAligned() {
    try {
      if (Math.abs(getTX() - VisionConstants.AlgaeAngle) < VisionConstants.AlgaeTolerance
          && getClosestFiducial().distToRobot < 1.5) {
        return true;
      }
    } catch (Exception e) {
      return false;
    }
    return false;

  }

  // Get values
  //Of closest
  public double getTX() {
    return LimelightHelpers.getTX(VisionConstants.LIMELIGHT_NAME);

  }
//Of closest
  public double getTY() {
    return LimelightHelpers.getTY(VisionConstants.LIMELIGHT_NAME);
  }
//Of closest
  public Angle getTXAngle() {
    return Angle.ofBaseUnits(LimelightHelpers.getTX(VisionConstants.LIMELIGHT_NAME), Degrees);
  }
//Of closest
  public Angle getTYAngle() {
    return Angle.ofBaseUnits(LimelightHelpers.getTY(VisionConstants.LIMELIGHT_NAME), Degrees);
  }
  //Of closest
  public double getTA() {
    return LimelightHelpers.getTA(VisionConstants.LIMELIGHT_NAME);
  }

  public boolean getTV() {
    return LimelightHelpers.getTV(VisionConstants.LIMELIGHT_NAME);
  }

  public double getClosestTX() {
    return getClosestFiducial().txnc;
  }

  public double getClosestTY() {
    return getClosestFiducial().tync;
  }

  public double getClosestTA() {
    return getClosestFiducial().ta;
  }
  //Returns value on an id
  public double getID_TX(int ID) {
    return getFiducialWithId(ID).txnc;
  }

  public double getID_TY(int ID) {
    return getFiducialWithId(ID).tync;
  }
}
