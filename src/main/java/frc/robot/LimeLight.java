package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;

public class LimeLight {
    FieldObject2d limelightFieldObject;
    Pose2d pose;
    public LimeLight(Field2d field) {
        limelightFieldObject = field.getObject("LimeLight");
    }

    public static double getTv() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0);
    }
    public static double getTx() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
    }
    public static double getTy() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
    }
    public static double getTa() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0.0);
    }
    public static double getCurrentPipeline() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("getpipe").getDouble(0.0);
    }

    public static void setCurrentPipeline(int pipelineId) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipelineId);
    }

    //simulation
    public void render(Pose2d position) {
        this.pose = position;
        limelightFieldObject.setPose(position);
    }

    private double getDegreeDifference() {
        Translation2d position = this.pose.getTranslation();
        Translation2d limelightToHub = Constants.hubPosition.minus(position);
        Translation2d headingVector = VectorUtil.moveForward(this.pose, 1.).getTranslation().minus(position);
        double dot = VectorUtil.dot(VectorUtil.unit(headingVector), VectorUtil.unit(limelightToHub));
        return Units.radiansToDegrees(Math.acos(dot));
    }

    public double getSimTv() {
        double state = 0.0;
        double degreeDifference = getDegreeDifference();
        if (Math.abs(degreeDifference) <= Constants.limelightHorizontalRange) {
            state = 1.0;
        }
        return state;
    }

    public double getSimTx(double defaultValue) {
        double degreeDifference = getDegreeDifference();
        return Math.abs(degreeDifference) <= Constants.limelightHorizontalRange ? degreeDifference : defaultValue;
    }

    public double getSimDistanceToHub() {
        return VectorUtil.getMagnitude(Constants.hubPosition.minus(this.pose.getTranslation()));
    }

}