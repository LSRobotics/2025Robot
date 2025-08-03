package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ShooterIOSparkFlex implements ShooterIO{
    private final SparkFlex shooterMotor = new SparkFlex(ShooterConstants.shooterMotorID, MotorType.kBrushless);

    public ShooterIOSparkFlex() {
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.appliedVolts = shooterMotor.getAppliedOutput() * shooterMotor.getBusVoltage();
        inputs.motorOutputPercent = shooterMotor.getAppliedOutput();
        inputs.velocityRPM = shooterMotor.getEncoder().getVelocity() * 60.0; // Convert to RPM
        inputs.shooterSpeed = shooterMotor.get();
    }

    @Override
    public void setVoltage(double volts) {
        shooterMotor.setVoltage(volts);
    }
    @Override
    public void setSpeed(double percent) {
        shooterMotor.set(percent);
        
    }

    @Override
    public void stop() {
        shooterMotor.stopMotor();
    }

}
