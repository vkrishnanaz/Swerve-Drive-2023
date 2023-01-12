package frc4146.robot.commands;

import common.drivers.Gyroscope;
import common.math.Rotation2;
import common.math.Vector2;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4146.robot.subsystems.DrivetrainSubsystem;

public class TurnRobotCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final Gyroscope gyro;
    private final double speed;
    private final double turnAmt;

    private double dampening = 1;
    private Rotation2 startAng;
    private ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    public TurnRobotCommand(DrivetrainSubsystem drivetrain, Gyroscope gyro, double turnAmt, double speed) {
        this.drivetrain = drivetrain;
        this.gyro = gyro;
        this.turnAmt= turnAmt;
        this.speed=speed;
    }

    @Override
    public void initialize() {
        this.startAng = gyro.getAngle();
        tab.addNumber("Angle Offset Error", ()->Math.abs(getOffset()/turnAmt));
    }
    @Override
    public void execute() {
        double offset = getOffset();
        double multiplier = 0.1+Math.abs(Math.sin(Math.sqrt(Math.abs(offset/turnAmt))));
        double adjSpeed = dampening * speed * multiplier;
        drivetrain.drive(Vector2.ZERO, Math.copySign(adjSpeed,offset), false);
    }
    @Override
    public boolean isFinished() {
        if(Math.abs(getOffset()) < 0.5 || (Math.abs(getOffset()/turnAmt) < 0.1 && Math.abs(getOffset()) < 1)) {
            if (drivetrain.getAngularVelocity() < 1) return true;
            else dampening *= 0.75;
        }
        return false;

    }
    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(Vector2.ZERO, 0.0, false);
        initialize();
    }

    public double getOffset() {
        return startAng.rotateBy(gyro.getAngle().inverse()).toDegrees() + turnAmt;
    }

}
