package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class VectorUtil {
    public static Pose2d moveForward(Pose2d pose, double offset) {
        Translation2d origin = pose.getTranslation();
        Rotation2d rotation = pose.getRotation();
        Translation2d offsetPoint = new Translation2d(rotation.getCos()*offset, rotation.getSin()*offset);
        return new Pose2d(origin.plus(offsetPoint), rotation);
    }
}
