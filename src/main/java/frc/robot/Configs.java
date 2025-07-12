package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Configs {
    public static final class ElevatorConfigs{
        public static final SparkMaxConfig elevatorConfig = new SparkMaxConfig();

        static {
            elevatorConfig.idleMode(IdleMode.kBrake);

            
            elevatorConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(0.03)
                .d(0.007)
                .outputRange(-0.10, 0.15)
                .maxMotion
                .maxVelocity(2000)
                .maxAcceleration(6000)
                .allowedClosedLoopError(0.1);
        }
    }
}
