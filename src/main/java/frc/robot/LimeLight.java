package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import frc.robot.subsystems.Drivetrain;

public class LimeLight {
    FieldObject2d limelightFieldObject;
    Pose2d pose;
    static PhotonCamera camera;
    //apriltag invasion >:)
    public LimeLight(Field2d field) {
        limelightFieldObject = field.getObject("LimeLight");
        camera = new PhotonCamera("gloworm");
    }

    public static double getTv() {
        boolean target = camera.getLatestResult().hasTargets();
        System.out.print(target);
        if (target == true) {
            return 1.0;
        } else {
            return 0.0;
        }
    }
    public static double getTx() {
        if (getTv() == 1.0) {
            var target = camera.getLatestResult().getBestTarget();
            return target.getYaw();
        } else {
            return 0.0;
        }
    }
    public static double getTy() {
        if (getTv() == 1.0) {
            var target = camera.getLatestResult().getBestTarget();
            return target.getPitch();
        } else {
            return 0.0;
        }
    }
    public static double getTa() {
        if (getTv() == 1.0) {
            var target = camera.getLatestResult().getBestTarget();
            return target.getArea();
        } else {
            return 0.0;
        }
    }
    public static double getCurrentPipeline() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("getpipe").getDouble(0.0);
    }

    public static void setCurrentPipeline(int pipelineId) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipelineId);
    }

    public static void setLedMode(int ledState) {
        //dont use this for turning off it is unreliable
        //NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(ledState);
    }

    //prereq is that tv has to be 1
    public static Object[] getRobotPoseFromVision() {
        Rotation2d lookVector = RobotContainer.drivetrainSubsystem.odometry.getEstimatedPosition().getRotation();
        double turretRotationsRelative = (RobotContainer.shooterSubsystem.turretAlignmentMotor.getSelectedSensorPosition()/2048.) * Constants.GEARING_turret;
        double rotationOffset = (turretRotationsRelative * 360.) - LimeLight.getTx();
        double theta = Units.degreesToRadians(rotationOffset + lookVector.getDegrees());
        double approachAngle = Units.degreesToRadians(Constants.limelightMountAngle + LimeLight.getTy());
        double camToHubDistHorizontal = (Constants.hubHeight - Constants.limelightHeight) / Math.tan(approachAngle);
        Translation2d robotDisplacementFromHub = new Translation2d(Math.cos(theta)*camToHubDistHorizontal, Math.sin(theta)*camToHubDistHorizontal);

        Object[] data = new Object[2];
        data[0] = new Pose2d(Constants.hubPosition.minus(robotDisplacementFromHub), lookVector);
        data[1] = camToHubDistHorizontal;
        return data;
    }

    //simulation
    public void render(Pose2d position) {
        this.pose = position;
        limelightFieldObject.setPose(position);
    }
    LineRenderer line = new LineRenderer(0,0,0,0,Drivetrain.field);
    public double getDegreeDifference() {
        Translation2d robotPose = RobotContainer.drivetrainSubsystem.odometry.getEstimatedPosition().getTranslation();
        Translation2d limelightToHub = Constants.hubPosition.minus(robotPose);
        double radian = Units.degreesToRadians(RobotContainer.shooterSubsystem.degreesOnTurret() + RobotContainer.drivetrainSubsystem.odometry.getEstimatedPosition().getRotation().getDegrees());/* RobotContainer.drivetrainSubsystem.gyro.getAngle()*/ //turret is fixed to robot rotation however gyro is inverted
        Translation2d originOrientation = new Translation2d(Math.cos(radian), Math.sin(radian));
        Translation2d localPosition = robotPose.plus(originOrientation);
        Translation2d headingVector = robotPose.minus(localPosition);
        line.update(robotPose.getX(), robotPose.getY(), localPosition.getX(), localPosition.getY(), Drivetrain.field);
        double directedAngleRadians = Math.atan2(headingVector.getY(), headingVector.getX()) - Math.atan2(limelightToHub.getY(), limelightToHub.getX());
        return -1*(180-Units.radiansToDegrees(directedAngleRadians));
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
        Pose2d odometryPose = RobotContainer.drivetrainSubsystem.odometry.getEstimatedPosition();
            double radian = Units.degreesToRadians(odometryPose.getRotation().getDegrees());
            Translation2d poseVector = new Translation2d(Math.cos(radian), Math.sin(radian));
            Translation2d robotToHub = Constants.hubPosition.minus(odometryPose.getTranslation());
            double degreeDifference = Math.atan2((poseVector.getY() * robotToHub.getX()) - (poseVector.getX() * robotToHub.getY()),
                    (poseVector.getX() * robotToHub.getX()) + (poseVector.getY() * robotToHub.getY()));
            degreeDifference = Units.radiansToDegrees(degreeDifference);
        return Math.abs(degreeDifference) <= Constants.limelightHorizontalRange ? degreeDifference : defaultValue;
    }

    public double getSimDistanceToHub() {
        return VectorUtil.getMagnitude(Constants.hubPosition.minus(this.pose.getTranslation()));
    }
}