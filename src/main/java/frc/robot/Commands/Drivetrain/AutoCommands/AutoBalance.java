package frc.robot.Commands.Drivetrain.AutoCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DrivetrainSubsystem;

public class AutoBalance extends CommandBase {
    DrivetrainSubsystem m_drivetrainSubsystem;
    
    PIDController m_rollPIDController;
    double m_yVelocity;

    public AutoBalance(DrivetrainSubsystem drivetrainSubsystem) {
        m_drivetrainSubsystem = drivetrainSubsystem;

        addRequirements(m_drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        m_rollPIDController = new PIDController(0.01, 0, 0);
    }

    /* IMPORTANT: Robot has to be approaching the charging station from the left side! */
    @Override
    public void execute() {
        m_yVelocity = MathUtil.applyDeadband(m_rollPIDController.calculate(m_drivetrainSubsystem.getRoll()), 0.01);
        m_drivetrainSubsystem.drive(0, m_yVelocity, 0, true);
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(m_drivetrainSubsystem.getRoll()) > 3 ) {
            return false;
        }
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(0, 0, 0, true);
    }
}
