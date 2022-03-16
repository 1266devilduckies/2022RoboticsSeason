package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.EncoderSetter;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class DriveSubsystem extends SubsystemBase {

    // Odometry class for tracking robot pose
    public static DifferentialDriveOdometry m_odometry;

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
        resetEncoders();
        m_odometry = new DifferentialDriveOdometry(RobotMap.gyro.getRotation2d());
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        SmartDashboard.putNumber("testing periodic", System.currentTimeMillis());
        m_odometry.update(
                RobotMap.gyro.getRotation2d(),
                EncoderSetter.nativeUnitsToDistanceMeters(RobotMap.MainLeftMotorBack.getSelectedSensorPosition(0)),
                EncoderSetter.nativeUnitsToDistanceMeters(RobotMap.MainRightMotorBack.getSelectedSensorPosition(0)));
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
        return new DifferentialDriveWheelSpeeds(
                EncoderSetter.nativeUnitsToDistanceMeters(RobotMap.MainLeftMotorBack.getSelectedSensorVelocity(0)),
                EncoderSetter.nativeUnitsToDistanceMeters(RobotMap.MainRightMotorBack.getSelectedSensorVelocity(0)));

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
        return (EncoderSetter.nativeUnitsToDistanceMeters(RobotMap.MainLeftMotorBack.getSelectedSensorPosition(0))
                + EncoderSetter.nativeUnitsToDistanceMeters(RobotMap.MainRightMotorBack.getSelectedSensorPosition(0)))
                / 2.0;
    }

    /*
     * public Encoder getLeftEncoder() {
     * return RobotMap.MainLeftMotorBack;
     * }
     * public Encoder getRightEncoder() {
     * // return RobotMap.MainRightMotorBack;
     * return null;
     * }
     */

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