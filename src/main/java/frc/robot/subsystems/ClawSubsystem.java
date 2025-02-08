package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.math.MathUtil;
import com.ctre.phoenix6.hardware.TalonFX;

public class ClawSubsystem extends SubsystemBase {
  private final TalonFX clawMotor = new TalonFX(ClawConstants.clawMotorID);
  private final TalonFX shooterMotor = new TalonFX(ClawConstants.shooterMotorID);
  private final DutyCycleEncoder clawEncoder;
  private static final double fullRange = 360;
  private static final double expectedZero = 0;

  public ClawSubsystem() {
    clawEncoder = new DutyCycleEncoder(1, fullRange, expectedZero);
    clawEncoder.setAssumedFrequency(975.6);
  }

  public double getClawPosition() {
    return MathUtil.clamp(clawEncoder.get(), expectedZero, fullRange);
  }
    
  public void runClawMotor(double speed) {
    clawMotor.set(speed);
  }

  public void runShooterMotor(double speed) {
    shooterMotor.set(speed);
  }
}
