package frc.robot;

import java.lang.reflect.Field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;

public class LimeLight {
    FieldObject2d limelightFieldObject;
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
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").setNumber(pipelineId);
    }

    //simulation
    public void render(Pose2d robotPosition) {
        limelightFieldObject.setPose(robotPosition);
    }

}