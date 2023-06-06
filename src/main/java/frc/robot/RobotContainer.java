package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.Commands.Default.DefaultDriveCommand;
import frc.robot.Commands.Default.DefaultElevatorCommand;
import frc.robot.Commands.Drivetrain.BrakeCommand;
import frc.robot.Commands.Drivetrain.IdleDriveCommand;
import frc.robot.Commands.Drivetrain.PositionDriveCommand;
import frc.robot.Commands.Drivetrain.AutoCommands.AutoBalance;
import frc.robot.Commands.Elevator.ElevatorPositionCommand;
import frc.robot.Commands.Intake.CloseIntakeCommand;
import frc.robot.Commands.Intake.OpenIntakeCommand;
import frc.robot.Commands.Limelight.ElevatorTargetTrackingWithLimelight;
import frc.robot.Commands.Limelight.LimelightAlignmentDriveCommand;
import frc.robot.Commands.Winch.WinchPositionCommand;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;
import frc.robot.Subsystems.WinchSubsystem;

/** Represents the entire robot. */
public class RobotContainer {
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final WinchSubsystem m_winchSubsystem = new WinchSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

  private final Joystick m_driveController = new Joystick(0);
  private final Joystick m_operatorController = new Joystick(1);

  private double m_powerLimit = 1.0;
  private static double m_rotatePower = 0;
  private static double m_winchDirection = 1;

  private boolean m_controllerEnabled = true;

  private final UsbCamera m_subsystemCamera;

  /**
   * This class stores all robot related subsystems, commands, and methods that
   * the {@link Robot} class can utilize during different OpModes.
   */
  public RobotContainer() {
    if (m_controllerEnabled) {
      m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
          m_drivetrainSubsystem,
          () -> -MathUtil.applyDeadband(m_driveController.getRawAxis(1), 0.05) * m_powerLimit
              * DrivetrainSubsystem.kMaxSpeed,
          () -> -MathUtil.applyDeadband(m_driveController.getRawAxis(0), 0.05) * m_powerLimit
              * DrivetrainSubsystem.kMaxSpeed,
          () -> (-MathUtil.applyDeadband(m_driveController.getRawAxis(4), 0.05) / 2.0) * m_powerLimit
              * DrivetrainSubsystem.kMaxAngularSpeed));
    } else {
      m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
          m_drivetrainSubsystem,
          () -> -MathUtil.applyDeadband(m_driveController.getRawAxis(1), 0.05) * (-m_driveController.getRawAxis(3) + 1)
              * DrivetrainSubsystem.kMaxSpeed,
          () -> -MathUtil.applyDeadband(m_driveController.getRawAxis(0), 0.05) * (-m_driveController.getRawAxis(3) + 1)
              * DrivetrainSubsystem.kMaxSpeed,
          () -> m_rotatePower * (-m_driveController.getRawAxis(3) + 1) * DrivetrainSubsystem.kMaxAngularSpeed));
    }

    m_elevatorSubsystem.setDefaultCommand(new DefaultElevatorCommand(
        m_elevatorSubsystem,
        m_winchSubsystem,
        () -> -MathUtil.applyDeadband(m_operatorController.getRawAxis(5), 0.05),
        () -> m_winchDirection * -MathUtil.applyDeadband(m_operatorController.getRawAxis(1), 0.05)));

    m_subsystemCamera = CameraServer.startAutomaticCapture(0);
    m_subsystemCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

    configureButtons();
  }

  // Currently used for testing kinematics
  public SequentialCommandGroup autonomousCommands() {
    return new SequentialCommandGroup(
        new PositionDriveCommand(m_drivetrainSubsystem, 4.0, 1.0, Math.toRadians(90), 3.50, Math.toRadians(120)),
        new PositionDriveCommand(m_drivetrainSubsystem, 3.0, 0.0, Math.toRadians(45), 3.50, Math.toRadians(120)));
  }

  private void configureButtons() {
    Button m_cancelPresets = new Button(() -> m_operatorController.getRawButton(8));
    m_cancelPresets.whenPressed(() -> cancelElevatorCommands());

    Button m_resetElevator = new Button(
        () -> m_operatorController.getRawButton(7));
    m_resetElevator.whenPressed(() -> reset());

    Button m_reverseWinch = new Button(() -> m_operatorController.getRawButton(6));
    m_reverseWinch.whileHeld(() -> reverseWinch(-1));
    m_reverseWinch.whenReleased(() -> reverseWinch(1));

    Button m_cross = new Button(() -> m_driveController.getRawButton(9));
    m_cross.whileHeld(new DefaultDriveCommand(m_drivetrainSubsystem, () -> 0, () -> 0, () -> 0.01));
    m_cross.whenReleased(() -> m_drivetrainSubsystem.getCurrentCommand().cancel());

    Button m_lowPosition = new Button(() -> m_operatorController.getRawButton(1));
    m_lowPosition.whenPressed(new ParallelCommandGroup(new ElevatorPositionCommand(m_elevatorSubsystem, "LOW", 0.8),
        new WinchPositionCommand(m_winchSubsystem, "IN", 0.8)));

    Button m_midPosition = new Button(() -> m_operatorController.getRawButton(3));
    m_midPosition.whenPressed(new ParallelCommandGroup(new ElevatorPositionCommand(m_elevatorSubsystem, "MID", 0.8),
        new WinchPositionCommand(m_winchSubsystem, "OUT", 0.8)));

    Button m_highPosition = new Button(() -> m_operatorController.getRawButton(4));
    m_highPosition.whenPressed(new ParallelCommandGroup(new ElevatorPositionCommand(m_elevatorSubsystem, "HIGH", 0.8),
        new WinchPositionCommand(m_winchSubsystem, "OUT", 0.8)));

    Button m_drivePosition = new Button(() -> m_operatorController.getRawButton(2));
    m_drivePosition.whenPressed(new ParallelCommandGroup(new ElevatorPositionCommand(m_elevatorSubsystem, "LOW", 0.8),
        new WinchPositionCommand(m_winchSubsystem, "DRIVE", 0.8)));

    Button m_openClaw = new Button(() -> m_operatorController.getRawAxis(2) > 0.5);
    m_openClaw.whenPressed(new OpenIntakeCommand(m_intakeSubsystem));

    Button m_closeClaw = new Button(() -> m_operatorController.getRawAxis(3) > 0.5);
    m_closeClaw.whenPressed(new CloseIntakeCommand(m_intakeSubsystem));

    Button m_rotateLeft = new Button(() -> m_driveController.getRawButton(11));
    m_rotateLeft.whileHeld(() -> setRotatePower("left"));
    m_rotateLeft.whenReleased(() -> setRotatePower("none"));

    Button m_rotateRight = new Button(() -> m_driveController.getRawButton(12));
    m_rotateRight.whileHeld(() -> setRotatePower("right"));
    m_rotateRight.whenReleased(() -> setRotatePower("none"));

    // Driver button A
    Button m_resetPose = new Button(() -> m_driveController.getRawButton(1));
    m_resetPose.whenPressed(() -> setPose(0, 0, 0));

    if (m_controllerEnabled) {
      // Driver button X
      Button m_brake = new Button(() -> m_driveController.getRawButton(3));
      m_brake.whenPressed(new BrakeCommand(m_drivetrainSubsystem));
      m_brake.whenReleased(() -> m_drivetrainSubsystem.getCurrentCommand().cancel());

      // Driver D-pad up
      Button m_incrementPowerLimit = new Button(
          () -> (m_driveController.getPOV() >= 315 || m_driveController.getPOV() <= 45));
      m_incrementPowerLimit.whenPressed(() -> changePowerLimit(0.2));

      // Driver D-pad down
      Button m_decrementPowerLimit = new Button(
          () -> (m_driveController.getPOV() >= 135 && m_driveController.getPOV() <= 225));
      m_decrementPowerLimit.whenPressed(() -> changePowerLimit(-0.2));
    } else {
      Button m_brake = new Button(() -> m_driveController.getRawButton(10));
      m_brake.whenPressed(new BrakeCommand(m_drivetrainSubsystem));
      m_brake.whenReleased(() -> m_drivetrainSubsystem.getCurrentCommand().cancel());
    }

    Button m_translationalLimelightTracking = new Button(() -> m_driveController.getRawButton(2));
    m_translationalLimelightTracking.whileActiveContinuous(
        new LimelightAlignmentDriveCommand(m_drivetrainSubsystem, m_limelightSubsystem, "translational"));
    m_translationalLimelightTracking.whenReleased(() -> m_drivetrainSubsystem.getCurrentCommand().cancel());

    Button m_rotationalLimelightTracking = new Button(() -> m_driveController.getRawButton(4));
    m_rotationalLimelightTracking.whileActiveContinuous(
        new LimelightAlignmentDriveCommand(m_drivetrainSubsystem, m_limelightSubsystem, "rotational"));
    m_rotationalLimelightTracking.whenReleased(() -> m_drivetrainSubsystem.getCurrentCommand().cancel());

    Button m_autoBalance = new Button(() -> m_driveController.getRawButton(5));
    m_autoBalance.whileActiveContinuous(new AutoBalance(m_drivetrainSubsystem));
    m_autoBalance.whenReleased(() -> m_drivetrainSubsystem.getCurrentCommand().cancel());

    Button m_elevatorLimelightTracking = new Button(() -> m_operatorController.getRawButton(5));
      m_elevatorLimelightTracking.whileActiveContinuous(new ElevatorTargetTrackingWithLimelight(m_winchSubsystem, m_elevatorSubsystem, m_limelightSubsystem, true, true));
      m_elevatorLimelightTracking.whenReleased(() -> m_drivetrainSubsystem.getCurrentCommand().cancel());

  }

  private void cancelElevatorCommands() {
    m_elevatorSubsystem.getCurrentCommand().cancel();
    m_winchSubsystem.getCurrentCommand().cancel();
  }

  public void reverseWinch(double direction) {
    m_winchDirection = direction;
  }

  public void setRotatePower(String state) {
    if (state.equals("left")) {
      m_rotatePower = 0.15;
    } else if (state.equals("right")) {
      m_rotatePower = -0.15;
    } else {
      m_rotatePower = 0;
    }
  }

  public void reset() {
    m_elevatorSubsystem.resetEncoders();
    m_winchSubsystem.resetEncoders();
  }

  public void setPose(double xPos, double yPos, double theta) {
    m_drivetrainSubsystem.setPose(xPos, yPos, theta);
  }

  public void setIdleMode(String idleMode) {
    m_drivetrainSubsystem.setIdleMode(idleMode);
  }

  private void changePowerLimit(double delta) {
    if ((m_powerLimit <= 1.0 - Math.abs(delta) || delta <= 0) && (m_powerLimit >= Math.abs(delta) || delta >= 0)) {
      m_powerLimit += delta;
    }
  }
}
