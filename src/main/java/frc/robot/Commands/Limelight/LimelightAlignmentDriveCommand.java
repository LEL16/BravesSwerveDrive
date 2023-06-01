package frc.robot.Commands.Limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;

public class LimelightAlignmentDriveCommand extends CommandBase {
    private final LimelightSubsystem m_limelightSubsystem;
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private PIDController m_yPIDController;
    private PIDController m_xPIDController;
    private PIDController m_angController;

    private double m_yVelocity;
    private double m_xVelocity;
    private double m_angularVelocity;
    private double m_distanceToRobot;
    private double m_deadband = 0.005;
    private double m_kP = 0.025;
    private double m_isFinishedTolerance = 0.05;
    private String m_yControlMode;

    /**
     * @param drivetrainSubsystem The robot's drivetrain subsystem
     * @param limelightSubsystem  The robot's Limelight subsystem
     * @param distanceToRobot    The desired distance to maintain from the player
     * @param yControlMode        The type of correctional movement in the y direction: "translational" or "rotational"
     */
    public LimelightAlignmentDriveCommand(DrivetrainSubsystem drivetrainSubsystem, LimelightSubsystem limelightSubsystem, String yControlMode, double distanceToRobot) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_limelightSubsystem = limelightSubsystem;

        m_distanceToRobot = distanceToRobot;
        m_yControlMode = yControlMode;

        addRequirements(m_drivetrainSubsystem, m_limelightSubsystem);
    }

    public void initialize() {
        m_yPIDController = new PIDController(m_kP, 0, 0);
        m_xPIDController = new PIDController(m_kP, 0, 0);
        m_angController = new PIDController(m_kP, 0, 0);
    }

    public void execute() {
        if (m_yControlMode.equals("translational")) { m_yVelocity = m_yPIDController.calculate(m_limelightSubsystem.getYTargetAngle()); } 
        else { m_angularVelocity = m_angController.calculate(m_limelightSubsystem.getYTargetAngle()); }

        m_xVelocity = -m_xPIDController.calculate((m_limelightSubsystem.getDistance() - m_distanceToRobot) * 100);

        m_xVelocity = (Math.abs(m_xVelocity) < m_deadband) ? 0 : m_xVelocity;
        m_yVelocity = (Math.abs(m_yVelocity) < m_deadband) ? 0 : m_yVelocity;
        m_angularVelocity = (Math.abs(m_angularVelocity) < m_deadband) ? 0 : m_yVelocity;

        m_drivetrainSubsystem.drive(
                m_xVelocity,
                m_yVelocity,
                m_angularVelocity,
                false);
    }

    public boolean isFinished() {
        if (Math.abs(m_limelightSubsystem.getYTargetAngle()) > m_isFinishedTolerance || (Math.abs(m_limelightSubsystem.getDistance() - m_distanceToRobot) > m_isFinishedTolerance / 5.0)) { return false; }
        return true;
    }

    @Override
    public void end(boolean interrupted) { m_drivetrainSubsystem.drive(0, 0, 0, true); }
}