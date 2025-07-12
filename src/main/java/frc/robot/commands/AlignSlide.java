package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.ctre.phoenix6.swerve.SwerveRequest;

public class AlignSlide extends Command {
    private final CommandSwerveDrivetrain m_Swerve;
    private final double timeForward;
    private final double timeRight;
    private final double speed;
    private final Timer timer = new Timer();
    private boolean movingForward = true;

    private final SwerveRequest.RobotCentric swerveRequest = new SwerveRequest.RobotCentric();

    public AlignSlide(CommandSwerveDrivetrain swerve, double timeForward, double timeRight, double speed) {
        m_Swerve = swerve;
        this.timeForward = timeForward;
        this.timeRight = timeRight;
        this.speed = speed;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if (timer.get() < timeForward) {
            // Move forward
            m_Swerve.setControl(swerveRequest.withVelocityX(speed).withVelocityY(0).withRotationalRate(0));
        } else if (movingForward) {
            // Transition to moving right
            movingForward = false;
            timer.reset();
            timer.start();
        } else if (timer.get() < timeRight) {
            // Move right
            m_Swerve.setControl(swerveRequest.withVelocityX(0).withVelocityY(speed).withRotationalRate(0));
        }
    }

    @Override
    public boolean isFinished() {
        return !movingForward && timer.get() >= timeRight;
    }

    @Override
    public void end(boolean interrupted) {
        m_Swerve.setControl(swerveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
        timer.stop();
    }
}
