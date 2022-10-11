package frc.robot.commands.simple;

import org.opencv.core.Mat;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.LimeLight;
import frc.robot.Robot;
import frc.robot.VectorUtil;
import frc.robot.subsystems.Drivetrain;

public class AlignToTarget extends CommandBase {
    Drivetrain drivetrainSubsystem;
    double error;
    double kP = 0.2;
    double kI = 0.02;
    double kD = 0.0;
    PIDController skidAnglePID = new PIDController(kP, kI, kD);

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

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double canSeeAnyTarget = Robot.isReal() ? LimeLight.getTv()
                : drivetrainSubsystem.limelightSim.getSimTv();
        error = Robot.isReal() ? LimeLight.getTx()
        : drivetrainSubsystem.limelightSim.getSimTx(0.0);
        double pidOutput = skidAnglePID.calculate(-error, 0.0);
        if (canSeeAnyTarget == 0.0) {
            Pose2d odometryPose = drivetrainSubsystem.odometry.getEstimatedPosition();
            double radian = Units.degreesToRadians(odometryPose.getRotation().getDegrees());
            Translation2d poseVector = new Translation2d(Math.cos(radian), Math.sin(radian));
            Translation2d robotToHub = Constants.hubPosition.minus(odometryPose.getTranslation());
            error = Math.atan2((poseVector.getY() * robotToHub.getX()) - (poseVector.getX() * robotToHub.getY()),
                    (poseVector.getX() * robotToHub.getX()) + (poseVector.getY() * robotToHub.getY()));

            error = Units.radiansToDegrees(error);
            pidOutput = skidAnglePID.calculate(-error, 0.0);
        }
        pidOutput += Constants.kSLinear;
        drivetrainSubsystem.tankDriveVolts(pidOutput, -pidOutput);
        System.out.println(error);
    }

    @Override
    public boolean isFinished() {
        return error < 0.5;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
