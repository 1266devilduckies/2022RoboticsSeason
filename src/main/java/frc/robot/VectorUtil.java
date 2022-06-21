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

    public static double dot(Translation2d a, Translation2d b) {
        return a.getX() * b.getX() + a.getY() * b.getY();
    }
    public static double getMagnitude(Translation2d vector) {
        return Math.sqrt(dot(vector, vector));
    }
    public static Translation2d unit(Translation2d vector) {
        double mag = getMagnitude(vector);
        return new Translation2d(vector.getX()/mag, vector.getY()/mag);
    }
}
