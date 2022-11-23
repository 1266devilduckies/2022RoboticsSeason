package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Drivetrain;

public class Trilaterator {
    //https://www.desmos.com/calculator/d2aa8qxe1g

    private static double lineEqu(double x, double x1, double x2, double y1, double y2, double r1, double r2) {
        double numerator = x1*x1+y1*y1+r2*r2-x2*x2-y2*y2-r1*r1;
        double subtractPart1 = numerator/(2.*(y1-y2));
        double subtractPart2 = (x*(x1-x2))/(y1-y2);
        return subtractPart1 - subtractPart2;
    }
    //cameraPitch is ty
    public static Translation2d findOrigin(Translation3d a, Translation3d b, double cameraPitch, boolean debugFlag, double r1, double r2) {
        double x1 = a.getX();
        double x2 = b.getX();
        double y1 = a.getY();
        double y2 = b.getY();

        double pitch = cameraPitch + Constants.limelightMountAngle;

        if (!debugFlag) {
            r1 = (a.getZ() - Constants.limelightHeight)/Math.tan(Units.degreesToRadians(pitch));
            r2 = (b.getZ() - Constants.limelightHeight)/Math.tan(Units.degreesToRadians(pitch));
        }

        double gradient = (x1-x2)/(y1-y2);
        double kA = 1 + gradient*gradient;
        double num1 = x1*x1-y1*y1+r2*r2-x2*x2-y2*y2-r1*r1+2.*y1*y2;
        double frac1 = num1/(y1-y2);
        double frac2 = frac1 * gradient;
        double kB = -(2*x1+frac2);
        double frac3 = num1/(2.*(y1-y2));
        double kC = x1*x1-r1*r1+frac3*frac3;
        System.out.println(kA + ", " + kB + ", " + kC);

        double s1 = (-kB + Math.sqrt(kB*kB-4.*kA*kC))/(2.*kA);
        double s2 = (-kB - Math.sqrt(kB*kB-4.*kA*kC))/(2.*kA);
        Translation2d p1 = new Translation2d(
            s1, lineEqu(s1,x1,x2,y1,y2,r1,r2)
        );
        Translation2d p2 = new Translation2d(
            s2, lineEqu(s2,x1,x2,y1,y2,r1,r2)
        );

        //Translation2d currPosOdo = 

        return p1;
    }
}
