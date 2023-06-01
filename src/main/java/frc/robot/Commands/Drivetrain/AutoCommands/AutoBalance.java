package frc.robot.Commands.Drivetrain.AutoCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DrivetrainSubsystem;

public class AutoBalance extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    
    private final PIDController m_rollPIDController;
    private final PIDController m_pitchPIDController;
    private static final double m_kP = 0.005;
    private static final double m_kI = 0.0;
    private static final double m_kD = 0.0;
    private static final double m_degreesTolerance = 1.0;

    private double m_yVelocity;
    private double m_xVelocity;

    public AutoBalance(DrivetrainSubsystem drivetrainSubsystem) {
        m_drivetrainSubsystem = drivetrainSubsystem;

        m_rollPIDController = new PIDController(m_kP, m_kI, m_kD);
        m_pitchPIDController = new PIDController(m_kP, m_kI, m_kD);

        m_rollPIDController.setTolerance(m_degreesTolerance);
        m_pitchPIDController.setTolerance(m_degreesTolerance);

        addRequirements(m_drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        m_rollPIDController.setSetpoint(0);
        m_pitchPIDController.setSetpoint(0);
    }

    @Override
    public void execute() {
        m_xVelocity = MathUtil.applyDeadband(m_pitchPIDController.calculate(m_drivetrainSubsystem.getPitch()), 0.01);
        m_yVelocity = MathUtil.applyDeadband(m_rollPIDController.calculate(m_drivetrainSubsystem.getRoll()), 0.01);
        m_drivetrainSubsystem.drive(-m_xVelocity, m_yVelocity, 0, true);
    }

    @Override
    public boolean isFinished() {
        return m_rollPIDController.atSetpoint() && m_pitchPIDController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(0, 0, 0, true);
    }
}
