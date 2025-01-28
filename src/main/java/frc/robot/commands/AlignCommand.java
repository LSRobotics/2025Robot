package frc.robot.commands;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.VisionSubsystem;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.units.measure.*;

import frc.robot.Constants.VisionConstants;

public class AlignCommand extends Command {
    private final VisionSubsystem m_Vision;
    private final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> m_Swerve;
    private final Distance holdDistance;
    private final int pipelineID;
    private final SwerveRequest.FieldCentric m_alignRequest;

    private final ProfiledPIDController aimController;
    private final ProfiledPIDController rangeController;

    //From tunerconsts and robtocontainer.java
    private static final double MAX_AIM_VELOCITY = 1.5*Math.PI; // radd/s
    private static final double MAX_AIM_ACCELERATION = Math.PI / 2; // rad/s^2
    private static final double MAX_RANGE_VELOCITY = 1.0; // m/s
    private static final double MAX_RANGE_ACCELERATION = 0.5; // m/2^s
  
    // Todo - Tune later
    private static final double AIM_P = 0.1; //Proprotinal
    private static final double AIM_I = 0.01; //Gradual corretction
    private static final double AIM_D = 0.05; //Smooth oscilattions
    
    private static final double RANGE_P = 0.1;
    private static final double RANGE_I = 0.01;
    private static final double RANGE_D = 0.05;

    public AlignCommand(VisionSubsystem vision, SwerveDrivetrain<TalonFX, TalonFX, CANcoder> swerve, Distance holdDistance, int pipelineID) {
        m_Vision = vision;
        m_Swerve = swerve;
        this.holdDistance = holdDistance;
        this.pipelineID = pipelineID;
        
        this.m_alignRequest = new SwerveRequest.FieldCentric().withDeadband(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.1).withRotationalDeadband(0.1);

        aimController = new ProfiledPIDController(AIM_P, AIM_I, AIM_D, new TrapezoidProfile.Constraints(MAX_AIM_VELOCITY, MAX_AIM_ACCELERATION));

        aimController.enableContinuousInput(-Math.PI, Math.PI); //Wrpa from -pi to ip
        
        rangeController = new ProfiledPIDController(RANGE_P, RANGE_I, RANGE_D, new TrapezoidProfile.Constraints(MAX_RANGE_VELOCITY, MAX_RANGE_ACCELERATION));

        addRequirements(m_Vision);
    }

    @Override
    public void initialize() {
        m_Vision.setPipelineIndex(pipelineID);
        
        aimController.reset(0);
        rangeController.reset(m_Vision.getDistance(this.pipelineID,VisionConstants.REEF_APRILTAG_HEIGHT.in(Inches)).in(Inches)); //Init dist
        
        aimController.setGoal(0); // tx=0 is centered
        rangeController.setGoal(holdDistance.in(Meters));
    }

    @Override
    public void execute() {
        Angle tx = Angle.ofBaseUnits(m_Vision.getTX(),Degrees);
        Distance currentDistance = m_Vision.getDistance(this.pipelineID,VisionConstants.REEF_APRILTAG_HEIGHT.in(Inches));

        double rotationOutput = aimController.calculate(tx.in(Radians));
        double rangeOutput = rangeController.calculate(currentDistance.in(Meters));

        Translation2d translation = new Translation2d(rangeOutput, 0);
                
        m_Swerve.setControl(m_alignRequest
            .withVelocityX(translation.getX())
            .withVelocityY(translation.getY())
            .withRotationalRate(rotationOutput));
    }
    
    @Override
    public void end(boolean interrupted) {
        m_Swerve.setControl(m_alignRequest
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        return aimController.atGoal() && rangeController.atGoal();
    }

    public AlignCommand withTolerance(Angle aimTolerance, Distance rangeTolerance) {
        aimController.setTolerance(aimTolerance.in(Radians));
        rangeController.setTolerance(rangeTolerance.in(Meters));
        return this;
    }
}