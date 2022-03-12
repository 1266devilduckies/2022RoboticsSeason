package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class DriveSubsystem extends SubsystemBase {

    public static final class DriveConstants {
        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.2;

        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kPDriveVel = 8.5;

        // Reasonable baseline values for a RAMSETE follower in units of meters and
        // seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        public static final double kTrackwidthMeters = 0.71;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackwidthMeters);

        public static final int kEncoderCPR = 2048;
        public static final double kWheelDiameterMeters = 0.15;
        public static final double kEncoderDistancePerPulse = (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;
    }

    // Odometry class for tracking robot pose
    public static DifferentialDriveOdometry m_odometry;

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        RobotMap.MainRightMotorBack.setInverted(true);

        resetEncoders();
        m_odometry = new DifferentialDriveOdometry(RobotMap.gyro.getRotation2d());
        Robot.m_driveSim.update(0.001);
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        m_odometry.update(
                RobotMap.gyro.getRotation2d(),
                RobotMap.MainLeftMotorBack.getSelectedSensorPosition(0) / DriveConstants.kEncoderDistancePerPulse,
                RobotMap.MainRightMotorBack.getSelectedSensorPosition(0) / DriveConstants.kEncoderDistancePerPulse);
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(RobotMap.MainLeftMotorBack.getSelectedSensorPosition(0),
                RobotMap.MainRightMotorBack.getSelectedSensorPosition(0));
               
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, RobotMap.gyro.getRotation2d());
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void arcadeDrive(double fwd, double rot) {
        RobotMap.m_drive.arcadeDrive(fwd, rot);
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts  the commanded left output
     * @param rightVolts the commanded right output
     */

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        RobotMap.MainLeftMotorBack.setSelectedSensorPosition(0);
        RobotMap.MainRightMotorBack.setSelectedSensorPosition(0);
    }

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
        return (RobotMap.MainLeftMotorBack.getSelectedSensorPosition(0)
                + RobotMap.MainRightMotorBack.getSelectedSensorPosition(0)) / 2.0;
    }

    /**
     * Gets the left drive encoder.
     *
     * @return the left drive encoder
     */
    public Encoder getLeftEncoder() {
        // return RobotMap.MainLeftMotorBack;
        return null;
    }

    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    public Encoder getRightEncoder() {
        // return RobotMap.MainRightMotorBack;
        return null;
    }

    public DifferentialDrive getDifferentialDrive() {
        Robot.m_driveSim.update(0.001);
        return RobotMap.m_drive;
    }

    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more
     * slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        RobotMap.m_drive.setMaxOutput(maxOutput);
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        RobotMap.gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return RobotMap.gyro.getRotation2d().getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return -RobotMap.gyro.getRate();
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        RobotMap.MainLeftMotorBack.setVoltage(leftVolts);
        RobotMap.MainRightMotorBack.setVoltage(rightVolts);
        RobotMap.m_drive.feed();
    }
}