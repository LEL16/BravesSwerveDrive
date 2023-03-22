package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class TranslationDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final double m_distance;
    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;

    private final PIDController m_xDistanceController;
    private final PIDController m_yDistanceController;

    private final double xDistance;
    private final double yDistance;

    public TranslationDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               double distanceX,
                               double distanceY, 
                               double velocity) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_distance = Math.hypot(distanceX, distanceY);
        this.m_translationXSupplier = () -> (distanceX / this.m_distance) * velocity;
        this.m_translationYSupplier = () -> (distanceY / this.m_distance) * velocity;
        
        m_xDistanceController = new PIDController(0.1, 0, 0);
        m_yDistanceController = new PIDController(0.1, 0, 0);

        xDistance = distanceX;
        yDistance = distanceY;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        double xVelocity = m_translationXSupplier.getAsDouble();
        double yVelocity = m_translationYSupplier.getAsDouble();

        double xDistanceError = getXdistanceError();
        double yDistanceError = getYdistanceError();

        // Apply PID control to x and y distances to get velocities
        xVelocity = m_xDistanceController.calculate(xDistanceError);
        yVelocity = m_yDistanceController.calculate(yDistanceError);

        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        m_drivetrainSubsystem.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                    xVelocity,
                    yVelocity,
                    0,
                    m_drivetrainSubsystem.getGyroscopeRotation()
            )
        );
    }

    @Override
    public boolean isFinished() {
        if (m_drivetrainSubsystem.getDistanceTravelled() < m_distance) {
            return false;
        }
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        m_drivetrainSubsystem.updateDistance();
    }

    private double getXdistanceError() {
        return m_distance - xDistance;
    }

    private double getYdistanceError() {
        return m_distance - yDistance;
    }
}
