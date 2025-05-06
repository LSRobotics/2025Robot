package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClawSubsystem extends SubsystemBase {
  private final TalonFX clawMotor = new TalonFX(ClawConstants.clawMotorID);

  public ClawSubsystem() {

    TalonFXConfigurator config = clawMotor.getConfigurator();
    CurrentLimitsConfigs limitConfig = new CurrentLimitsConfigs();

   // limitConfig.StatorCurrentLimit = 100;
  //  limitConfig.StatorCurrentLimitEnable = true;
    config.apply(limitConfig);
                                         
    clawMotor.setNeutralMode(NeutralModeValue.Brake);
    clawMotor.setPosition(Angle.ofBaseUnits(0, Radian));
  }

  public Angle getClawPosition() {
    return Angle.ofBaseUnits(clawMotor.getPosition().getValue().in(Radian)%(2*Math.PI*ClawConstants.GEAR_RATIO), Radians);
  }
    
  public void runClawMotor(double speed) {
    clawMotor.set(speed);
  }

  public void motorVoltage(double voltage) {
    clawMotor.setVoltage(voltage);
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Claw Angle: ", getClawPosition().in(Radian));
  }
}