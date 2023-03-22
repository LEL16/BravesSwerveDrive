package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WinchSubsystem;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class DefaultElevatorCommand extends CommandBase {
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final WinchSubsystem m_winchSubsystem;

    private final DoubleSupplier m_elevatorPulleySpeed;
    private final DoubleSupplier m_winchSpeed;

    private final SlewRateLimiter LimElevatorSpeed, LimWinchSpeed;
    private int kElevatorExtendMaxAccelerationPerSecond;
    private int kWinchExtendMaxAccelerationPerSecond;

    public DefaultElevatorCommand(ElevatorSubsystem elevatorSubsystem, WinchSubsystem winchSubsystem, DoubleSupplier elevatorPulleySpeed, DoubleSupplier winchSpeed) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_winchSubsystem = winchSubsystem;
        m_elevatorPulleySpeed = elevatorPulleySpeed;
        m_winchSpeed = winchSpeed;

        kElevatorExtendMaxAccelerationPerSecond = 3;
        kWinchExtendMaxAccelerationPerSecond = 3;
        LimElevatorSpeed = new SlewRateLimiter(kElevatorExtendMaxAccelerationPerSecond);
        LimWinchSpeed = new SlewRateLimiter(kWinchExtendMaxAccelerationPerSecond);
        
        addRequirements(elevatorSubsystem, winchSubsystem);
    }

    @Override
    public void execute() {
        double l_elevatorSpeed = m_elevatorPulleySpeed.getAsDouble();
        double l_winchSpeed = m_winchSpeed.getAsDouble();

        l_winchSpeed = LimWinchSpeed.calculate(l_winchSpeed);
        l_elevatorSpeed = LimElevatorSpeed.calculate(l_elevatorSpeed);

        m_elevatorSubsystem.extend(l_elevatorSpeed);
        m_winchSubsystem.rotate(l_winchSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        m_elevatorSubsystem.extend(0);
        m_winchSubsystem.rotate(0);
    }
}
