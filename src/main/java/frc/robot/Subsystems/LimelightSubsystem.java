package frc.robot.Subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Represents a Limelight sensor */
public class LimelightSubsystem extends SubsystemBase {
    // NetworkTable to communicate with the Limelight device.
    private NetworkTable networkTable;

    // NetworkTableEntries to obtain various data from the Limelight device.
    private NetworkTableEntry pipelineEntry;
    private NetworkTableEntry camModeEntry;
    private NetworkTableEntry ledModeEntry;
    private NetworkTableEntry yEntry;
    private NetworkTableEntry xEntry;
    private NetworkTableEntry targetAreaEntry;
    private NetworkTableEntry foundTagEntry;

    // Shuffleboard tab specifically for the Limelight.
    private ShuffleboardTab limelightTab;

    // Shuffleboard data that can be manipulated.
    private GenericEntry ledStatusEntry;
    private GenericEntry cameraStatusEntry;
    private GenericEntry pipelineIdEntry;

    // Variables to store the current state of the Limelight.
    private boolean isCameraModeOn = false;
    private boolean isLedOn = true;

    // Variables to store the angles generated by the tracked object.
    private double yTargetAngle;
    private double xTargetAngle;
    private double targetArea;

    // Variables for object detection.
    private double foundTag;
    private boolean foundTagBool;

    // Variables for distance calculations.
    private boolean useTrigForDistanceCalc = false;
    private double targetAreaDistance;
    private double trigDistance;
    private double distance;

    /** The LimelightSubsystem constructor. Initializes the Limelight subsystem and sets up the NetworkTable. */
    public LimelightSubsystem() {
        networkTable = NetworkTableInstance.getDefault().getTable("limelight");    
    }

    @Override
    public void periodic() {
        // NetworkTableEntries to store the values generated by the Limelight.
        pipelineEntry = networkTable.getEntry("pipeline");
        camModeEntry = networkTable.getEntry("camMode");
        ledModeEntry = networkTable.getEntry("ledMode");
        yEntry = networkTable.getEntry("tx");
        targetAreaEntry = networkTable.getEntry("ta");
        xEntry = networkTable.getEntry("ty");
        foundTagEntry = networkTable.getEntry("tv");

        // Using the entries to generate data and/or change current Limelight
        // specifications.
        pipelineEntry.setNumber(Constants.PIPELINE_ID);
        if (isCameraModeOn) {
            camModeEntry.setNumber(1);
        } else {
            camModeEntry.setNumber(0);
        }
        if (isLedOn) {
            ledModeEntry.setNumber(3);
        } else {
            ledModeEntry.setNumber(1);
        }
        yTargetAngle = yEntry.getDouble(0.0);
        xTargetAngle = xEntry.getDouble(0.0);
        targetArea = targetAreaEntry.getDouble(0.0);
        foundTag = foundTagEntry.getDouble(0.0);
        foundTagBool = (foundTag != 0) ? true : false;

        // Using data points and trigonometry for distance calculations from the
        // Limelight to the object detected.
        targetAreaDistance = Units.inchesToMeters(54.4 * Math.pow(targetArea, -0.475));
        trigDistance = Units.inchesToMeters(((Constants.LIMELIGHT_GOAL_HEIGHT - Constants.LIMELIGHT_LENS_HEIGHT) * 100) / Math.tan(Math.toRadians(xTargetAngle) + Constants.LIMELIGHT_ANGLE));
        distance = (useTrigForDistanceCalc) ? trigDistance : targetAreaDistance;
    }

    public double getYTargetAngle() {
        return yTargetAngle;
    }

    public double getXTargetAngle() {
        return xTargetAngle;
    }

    public double getTargetAreaDistance() {
        return targetAreaDistance;
    }

    public double getTrigDistance() {
        return trigDistance;
    }

    public double getDistance() {
        return distance;
    }

    public boolean getFoundTag() {
        return foundTagBool;
    }

    public boolean getLEDStatus() {
        return isLedOn;
    }

    public boolean getCameraModeStatus() {
        return isCameraModeOn;
    }

    public void setLEDStatus(boolean isLedOn) {
        this.isLedOn = isLedOn;
    }

    public void setCameraStatus(boolean isCameraModeOn) {
        this.isCameraModeOn = isCameraModeOn;
    }
}