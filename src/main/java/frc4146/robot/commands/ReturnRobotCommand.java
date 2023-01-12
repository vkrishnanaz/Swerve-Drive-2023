package frc4146.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.ArrayList;

import common.math.Vector2;
import frc4146.robot.subsystems.DrivetrainSubsystem;

public class ReturnRobotCommand extends CommandBase {

  DrivetrainSubsystem drivetrain;

  ArrayList<Double> speeds;
  double rotationSpeed = 0;

  public ReturnRobotCommand(DrivetrainSubsystem drivetrain) {
    this.drivetrain = drivetrain;
  }

  @Override
  public void initialize() {
    speeds = drivetrain.speeds;
  }

  @Override
  public void execute() {
    if (speeds.size() == 0) {
      end(true);
    }
    rotationSpeed = -speeds.remove(0);

    drivetrain.drive(Vector2.ZERO, rotationSpeed, false);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(Vector2.ZERO, rotationSpeed, false);
  }
}
