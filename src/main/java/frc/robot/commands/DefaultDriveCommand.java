package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {

    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private int kTeleDriveMaxAccelerationUnitsPerSecond = 0;
    private int kTeleDriveMaxAngularAccelrationUnitsPerSecond = 0;

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        
        kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        kTeleDriveMaxAngularAccelrationUnitsPerSecond = 3;
        xLimiter = new SlewRateLimiter(kTeleDriveMaxAccelerationUnitsPerSecond);
        yLimiter = new SlewRateLimiter(kTeleDriveMaxAccelerationUnitsPerSecond);
        turningLimiter = new SlewRateLimiter(kTeleDriveMaxAngularAccelrationUnitsPerSecond);

        //SlewRateLimiters in order to limit the acceleration of the robot
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        double xSpeed = m_translationXSupplier.getAsDouble();
        double ySpeed = m_translationYSupplier.getAsDouble();
        double turningSpeed = m_rotationSupplier.getAsDouble();

        boolean fieldOriented = true;

        /* Applying a deadband
        xSpeed = Math.abs(xSpeed) > 0.05 ? xSpeed : 0.0;
        ySpeed = Math.abs(xSpeed) > 0.05 ? xSpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > 0.05 ? turningSpeed : 0.0;
        */
        
        // Applying the SlewRateLimitrs for acceleration
        xSpeed = xLimiter.calculate(xSpeed);
        ySpeed = yLimiter.calculate(ySpeed);
        turningSpeed = turningLimiter.calculate(turningSpeed);

        // Constructing the desired chassis speeds
        ChassisSpeeds chassisSpeed;
        if (fieldOriented) {
            //Relative to the field
            chassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, m_drivetrainSubsystem.getGyroscopeRotation());
        } else {
            //Robot oriented
            chassisSpeed = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // Driving the robot
        m_drivetrainSubsystem.drive(
                chassisSpeed
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
