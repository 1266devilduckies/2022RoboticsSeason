package frc.robot;

import java.lang.reflect.Array;
import java.util.ArrayList;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;

public class LineRenderer {
    private FieldObject2d line;
    private static double lineNum = 0;
    public LineRenderer(double x1, double y1, double x2, double y2, Field2d field) { //in terms of meters relative to the bottom left of the field
        // Translation2d direction = new Translation2d(x2-x1, y2-y1);
        // Rotation2d orientation = new Rotation2d(direction.getX(), direction.getY());
        // TrajectoryConfig config = new TrajectoryConfig(1,1);
        // config.setReversed(true);
        // var trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(x1,y1, orientation),
        // new ArrayList<Translation2d>(),
        // new Pose2d(x2,y2, Rotation2d.fromDegrees(0)),
        // config);

        lineNum++;
        line = field.getObject("Line"+lineNum);
        // line.setTrajectory(trajectory);
    }
    public void update(double x1, double y1, double x2, double y2, Field2d field) {
        
        Translation2d direction = new Translation2d(x2-x1, y2-y1);

        Rotation2d orientation = new Rotation2d(direction.getX(), direction.getY());
        TrajectoryConfig config = new TrajectoryConfig(1,1);
        config.setReversed(false);
        var trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(x1,y1, orientation),
        new ArrayList<Translation2d>(),
        new Pose2d(x2,y2, orientation),
        config);
        
        line.setTrajectory(trajectory);
        //System.out.println(orientation.getDegrees());
    }
}
