package frc.robot.commands.simple;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.LimeLight;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

public class AlignToTarget extends CommandBase {
    Drivetrain drivetrainSubsystem;
    double error;
    double kP = 0.2;
    double kI = 0.02;
    double kD = 0.0;
    double lastSpeed = 0;
    double lastTime = Timer.getFPGATimestamp();
    private final TrapezoidProfile.Constraints m_constraints =
    //max velocity of 1.75 degrees a second with 0.75 degree a second acceleration maxed
    new TrapezoidProfile.Constraints(1.75, 0.75);
    private final ProfiledPIDController skidAnglePID =
    new ProfiledPIDController(kP, kI, kD, m_constraints, 0.02); //.02 is 50hz

    public AlignToTarget(Drivetrain subsystem) {
        drivetrainSubsystem = subsystem;
        SmartDashboard.putNumber("aligner kP", kP);
        SmartDashboard.putNumber("aligner kI", kI);
        SmartDashboard.putNumber("aligner kD", kD);

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable datatable = inst.getTable("SmartDashboard");

        datatable.getEntry("aligner kP").addListener(event -> {
            kP = event.getEntry().getValue().getDouble();
            skidAnglePID.setP(kP);
            System.out.println("PID Gains: " + kP + ", " + kI + ", " + kD);
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        datatable.getEntry("aligner kI").addListener(event -> {
            kI = event.getEntry().getValue().getDouble();
            skidAnglePID.setI(kI);
            System.out.println("PID Gains: " + kP + ", " + kI + ", " + kD);
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        datatable.getEntry("aligner kD").addListener(event -> {
            kD = event.getEntry().getValue().getDouble();
            skidAnglePID.setD(kD);
            System.out.println("PID Gains: " + kP + ", " + kI + ", " + kD);
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        skidAnglePID.setTolerance(0.2, 0.1);

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        //try and align it to 0 degree error with 0 velocity at that position
        TrapezoidProfile.State zerodState = new TrapezoidProfile.State(0.0, 0.0);
        skidAnglePID.setGoal(zerodState);
    }

    @Override
    public void execute() {
        double canSeeAnyTarget = LimeLight.getTv();
        error = -LimeLight.getTx();
        double pidOutput = skidAnglePID.calculate(error);
        // if (canSeeAnyTarget == 0.0) {
        //     Pose2d odometryPose = drivetrainSubsystem.odometry.getEstimatedPosition();
        //     double radian = Units.degreesToRadians(odometryPose.getRotation().getDegrees());
        //     Translation2d poseVector = new Translation2d(Math.cos(radian), Math.sin(radian));
        //     Translation2d robotToHub = Constants.hubPosition.minus(odometryPose.getTranslation());
        //     error = Math.atan2((poseVector.getY() * robotToHub.getX()) - (poseVector.getX() * robotToHub.getY()),
        //             (poseVector.getX() * robotToHub.getX()) + (poseVector.getY() * robotToHub.getY()));

        //     error = Units.radiansToDegrees(error);
        //     pidOutput = skidAnglePID.calculate(-error, 0.0);
        // }
        double ff = Constants.kSLinear;
        if (error > 0.2) {
          ff = -Constants.kSLinear;
        }
        
        drivetrainSubsystem.robotDrive.arcadeDrive(0, pidOutput + ff/RobotController.getBatteryVoltage());
        System.out.println(error);
    }

    @Override
    public boolean isFinished() {
        return skidAnglePID.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.tankDriveVolts(0, 0);
    }
}
