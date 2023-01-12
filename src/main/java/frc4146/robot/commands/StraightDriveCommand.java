package frc4146.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4146.robot.subsystems.DrivetrainSubsystem;
import common.drivers.Gyroscope;
import common.math.RigidTransform2;
import common.math.Vector2;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
public class StraightDriveCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final Gyroscope gyro;
    private final double speed;
    private final double distance;

    private double startAng;
    private RigidTransform2 startPos;
    private ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    public StraightDriveCommand(DrivetrainSubsystem drivetrain, Gyroscope gyro, double distance, double speed) {
        this.drivetrain= drivetrain;
        this.distance= distance;
        this.speed=speed;
        this.gyro=gyro;

        addRequirements(drivetrain);
    }
    @Override
    public void initialize() {
        this.startAng = gyro.getAngle().toDegrees();
        this.startPos = drivetrain.getPose();
        tab.addNumber("Percent Distance Traveled", ()->getDistance()/distance);
    }
    @Override
    public void execute() {

        double ratio = Math.abs(1-getDistance()/distance);
        double multiplier = 0.1+Math.abs(Math.sin(Math.sqrt(ratio)));
        double adjSpeed = speed * multiplier;

        //Should be very close to zero, unless the robot has drifted
        double angDifference = gyro.getAngle().toDegrees() - startAng;


        drivetrain.drive(new Vector2(-adjSpeed*Math.cos(startAng),
                        -adjSpeed*Math.sin(startAng)), -0.01*angDifference, false);
    }
    @Override
    public boolean isFinished() {
        return getDistance() >= distance;
    }
    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(Vector2.ZERO, 0.0, false);
        initialize();
    }

    public double getDistance() {
        return Math.abs(drivetrain.getPose().translation.length - startPos.translation.length);
    }



}
