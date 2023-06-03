package frc.robot.Commands.Limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.WinchSubsystem;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;

public class ElevatorTargetTrackingWithLimelight extends CommandBase {
    private final WinchSubsystem m_winchSubsystem;
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final LimelightSubsystem m_limelightSubsystem;

    private PIDController m_winchPidController;
    private PIDController m_elevatorPidController;

    private final double m_distanceToAprilTag = 1.0;

    private double m_winchStartAngle;
    private double m_winchPower;
    private double m_winchTargetAngle;

    private double m_elevatorStartPosition;
    private double m_elevatorTargetPosition;
    private double m_elevatorPower;

    private boolean enableWinchTracking;
    private boolean enableElevatorTracking;

    public ElevatorTargetTrackingWithLimelight(WinchSubsystem winchSubsystem, ElevatorSubsystem elevatorSubsystem,
            LimelightSubsystem limelightSubsystem, boolean enableWinchTracking, boolean enableElevatorTracking) {
        m_winchSubsystem = winchSubsystem;
        m_elevatorSubsystem = elevatorSubsystem;
        m_limelightSubsystem = limelightSubsystem;

        this.enableWinchTracking = enableWinchTracking;
        this.enableElevatorTracking = enableElevatorTracking;

        if (this.enableWinchTracking && this.enableElevatorTracking) {
            addRequirements(m_winchSubsystem, m_elevatorSubsystem);
        } else if (this.enableWinchTracking) {
            addRequirements(m_winchSubsystem);
        } else if (this.enableElevatorTracking) {
            addRequirements(m_elevatorSubsystem);
        }
    }

    @Override
    public void initialize() {
        m_winchPidController = new PIDController(0.015, 0, 0);
        m_elevatorPidController = new PIDController(0.015, 0, 0);

        m_winchStartAngle = m_winchSubsystem.getWinchAbsPosition();
        m_winchTargetAngle = m_limelightSubsystem.getYTargetAngle();

        m_elevatorStartPosition = m_elevatorSubsystem.getElevatorAbsPosition();
        m_elevatorTargetPosition = m_limelightSubsystem.getTargetAreaDistance() - m_distanceToAprilTag;

        m_elevatorPower = m_elevatorPidController.calculate(m_elevatorStartPosition, m_elevatorTargetPosition);
        m_winchPower = m_winchPidController.calculate(m_winchStartAngle, m_winchTargetAngle);
    }

    public void execute() {
        if (enableElevatorTracking && m_limelightSubsystem.getFoundTag()) {
            m_elevatorSubsystem.extend(Math.copySign(m_elevatorPower, m_elevatorTargetPosition));
        }
        if (enableWinchTracking && m_limelightSubsystem.getFoundTag()) {
            m_winchSubsystem.rotate(Math.copySign(m_winchPower, m_winchTargetAngle));
        }

    }

    @Override
    public boolean isFinished() {
        if (Math.abs(m_winchStartAngle - m_winchTargetAngle) > 3) {
            return false;
        }
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        if (enableWinchTracking) {
            m_winchSubsystem.rotate(0);
        }
        if (enableElevatorTracking) {
            m_elevatorSubsystem.extend(0);
        }
    }
}