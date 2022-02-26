package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class limeLightDataFetcher {
    // table storing limelight data
    private static NetworkTable m_table = NetworkTableInstance.getDefault().getTable("limelight");
    // values we need
    static double dx = 0.0;
    static double dy = 0.0;
    static double canSeeAnyTarget = 0.0;

    // fetches data from limelight camera
    public static void fetchData() {
        NetworkTableEntry tx = m_table.getEntry("tx");
        NetworkTableEntry ty = m_table.getEntry("ty");
        NetworkTableEntry tv = m_table.getEntry("tv");
        dx = tx.getDouble(0.0);
        dy = ty.getDouble(0.0);
        canSeeAnyTarget = tv.getDouble(0.0);
    }

    // returns the degrees horziontally from target
    public static double getdegRotationToTarget() {
        return dx;
    }

    // return the degrees vertically away from target
    public static double getdegVerticalToTarget() {
        return dy;
    }

    // returns wether target is in frame
    public static double seeIfTargetsExist() {
        return canSeeAnyTarget;
    }
}
