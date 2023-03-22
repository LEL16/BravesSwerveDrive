package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import com.kauailabs.navx.frc.AHRS;


public class BalancingCommand extends CommandBase {
    private final double Navmultiplier;
    private final double balanceTuner;
    private final DrivetrainSubsystem driveTrain;
    private final AHRS m_navx;

    public BalancingCommand(DrivetrainSubsystem driveTrain, AHRS m_navx) {
        this.driveTrain = driveTrain;
        this.m_navx = m_navx;

        Navmultiplier = 0.7;
        balanceTuner = 0;

        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.driveTrain);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double roll = m_navx.getRoll()-balanceTuner;
        SmartDashboard.putNumber("Pitch", m_navx.getPitch());
        SmartDashboard.putNumber("Roll", roll);
        SmartDashboard.putNumber("BalanceTuner", balanceTuner);
        if (roll > 0.5){
            driveTrain.drive(new ChassisSpeeds(Math.pow(roll, 0.25)*Navmultiplier, 0, 0));
            driveTrain.drive(new ChassisSpeeds(Math.pow(roll, 0.25)*Navmultiplier, 0, 0));
        } else if (roll < - 0.5){
            driveTrain.drive(new ChassisSpeeds(-Math.pow(Math.abs(roll), 0.25)*Navmultiplier,0, 0));
            driveTrain.drive(new ChassisSpeeds(-Math.pow(Math.abs(roll), 0.25)*Navmultiplier*DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 0.0, 0.0));
        }
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }


    @Override
    public void end(boolean interrupted) {

    }
}