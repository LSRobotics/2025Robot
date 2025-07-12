package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

public class ElevatorIOSparkMax implements ElevatorIO {
    private final SparkMax m_elevatorMotor1 = new SparkMax(ElevatorConstants.kMotorID, MotorType.kBrushless);
    private final SparkMax m_elevatorMotor2 = new SparkMax(ElevatorConstants.lMotorID, MotorType.kBrushless);

    private final RelativeEncoder elevatorEncoder = m_elevatorMotor1.getEncoder();
    private final DigitalInput elevatorLimit = new DigitalInput(0);

    public ElevatorIOSparkMax() {
        m_elevatorMotor1.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
        
        m_elevatorMotor2.configure(new SparkMaxConfig().follow(m_elevatorMotor1).idleMode(IdleMode.kBrake),
        ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        elevatorEncoder.setPosition(0);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.position = elevatorEncoder.getPosition();
        inputs.velocityMetersPerSecond = elevatorEncoder.getVelocity();
        inputs.isZeroed = elevatorEncoder.getPosition() == 0;
        inputs.limitSwitchTriggered = !elevatorLimit.get();
        inputs.appliedVolts = m_elevatorMotor1.getAppliedOutput() * m_elevatorMotor1.getBusVoltage();
        inputs.currentAmps = m_elevatorMotor1.getOutputCurrent();
        inputs.motorOutputPercent = m_elevatorMotor1.getAppliedOutput();
        inputs.motorTemperatureCelsius = m_elevatorMotor1.getMotorTemperature();
    }

    @Override
    public void setVoltage(double volts) {
        m_elevatorMotor1.setVoltage(volts);
    }

    @Override
    public void setSpeed(double speed) {
        m_elevatorMotor1.set(speed);
    }

    @Override
    public void setEncoderPosition(double rotations) {
        elevatorEncoder.setPosition(rotations);
    }

    public void zeroEncoder() {
        elevatorEncoder.setPosition(0);
    }

    @Override
    public boolean getElevatorLimitSwitch() {
        return elevatorLimit.get();
    }

}
