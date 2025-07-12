package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.claw.ClawConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClawSubsystem extends SubsystemBase {
  private final ClawIO io;
  private final ClawIOInputsAutoLogged inputs = new ClawIOInputsAutoLogged();

  public ClawSubsystem(ClawIO io) {

    this.io = io;
  }

  public Angle getClawPosition() {
    return inputs.clawPosition;
  }
    
  public void runClawMotor(double speed) {
    io.setClawMotorSpeed(speed);
  }

  public void motorVoltage(double voltage) {
    io.setClawVoltage(voltage);
  }
  
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Claw", inputs);
  }
}