package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;

public class LineRenderer {
    private FieldObject2d line;
    private static double lineNum = 0;
    public LineRenderer(double x1, double y1, double x2, double y2, Field2d field) { //in terms of meters relative to the bottom left of the field
        lineNum++;
        line = field.getObject("Line"+lineNum);
    }
    public void update(double x1, double y1, double x2, double y2, Field2d field) {
        if (VectorUtil.getMagnitude(new Translation2d(x2-x1, y2-y1)) < Units.inchesToMeters(6)) { return; }
        Translation2d direction = new Translation2d(x2-x1, y2-y1);
        Rotation2d orientation = new Rotation2d(direction.getX(), direction.getY());
        line.setPoses(new Pose2d(x1,y1, orientation), new Pose2d(x2,y2,Rotation2d.fromDegrees(180).minus(orientation)));
    }
}
