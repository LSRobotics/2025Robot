package frc.robot.subsystems.claw;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ClawIOTalonFX implements ClawIO {
    private final TalonFX clawMotor = new TalonFX(ClawConstants.clawMotorID);
    private final StatusSignal<Angle> clawPosition = clawMotor.getPosition();
    private final StatusSignal<AngularVelocity> clawVelocity = clawMotor.getVelocity();
    private final StatusSignal<Current> clawMotorCurrent = clawMotor.getStatorCurrent();
    private final StatusSignal<Voltage> clawVoltage = clawMotor.getMotorVoltage();
    private final StatusSignal<AngularAcceleration> clawAcceleration = clawMotor.getAcceleration();
    private final StatusSignal<Temperature> clawMotorTemperature = clawMotor.getDeviceTemp();

    public ClawIOTalonFX() {
        TalonFXConfigurator config = clawMotor.getConfigurator();
        CurrentLimitsConfigs limitConfig = new CurrentLimitsConfigs();

        config.apply(limitConfig);

        clawMotor.setNeutralMode(NeutralModeValue.Brake);
        clawMotor.setPosition(Angle.ofBaseUnits(0, Radian));
    }

    @Override
    public void updateInputs(ClawIOInputs inputs){
        BaseStatusSignal.refreshAll(clawPosition, clawVelocity, clawMotorCurrent, clawVoltage, clawAcceleration, clawMotorTemperature);
        inputs.clawPosition = clawPosition.getValue();
        inputs.clawVoltage = clawVoltage.getValue().in(Volts);
        inputs.clawMotorCurrent = clawMotorCurrent.getValue().in(Amps);
        inputs.clawVelocity = clawVelocity.getValue().in(RadiansPerSecond);
        inputs.clawMotorAcceleration = clawAcceleration.getValue().in(RadiansPerSecondPerSecond);
        inputs.clawMotorTemperature = clawMotorTemperature.getValue();
    }

    @Override
    public void setClawMotorSpeed(double speed) {
        clawMotor.set(speed); 
    }

    @Override
    public void setClawVoltage(double voltage) {
        clawMotor.setControl(new VoltageOut(Volts.of(voltage)));
    }

}