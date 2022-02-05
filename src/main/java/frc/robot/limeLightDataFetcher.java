package frc.robot;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class limeLightDataFetcher {
    private static NetworkTable m_table = NetworkTableInstance.getDefault().getTable("limelight");
    static double dx = 0.0;
    static double dy = 0.0;
    public static void fetchData() {
        NetworkTableEntry tx = m_table.getEntry("tx");
        NetworkTableEntry ty = m_table.getEntry("ty");
        dx = tx.getDouble(0.0);
        dy = ty.getDouble(0.0);
    }
    public static double getdegRotationToTarget() {
        return dx;
    }
    public static double getdegVerticalToTarget() {
        return dy;
    }
}
