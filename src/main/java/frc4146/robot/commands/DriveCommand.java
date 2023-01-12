package frc4146.robot.commands;

import common.math.Vector2;
import common.robot.input.Axis;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4146.robot.subsystems.DrivetrainSubsystem;

/*
  Used with joysticks(or some Axis suppliers).
*/
public class DriveCommand extends CommandBase {
  private DrivetrainSubsystem drivetrainSubsystem;
  private Axis forward;
  private Axis strafe;
  private Axis rotation;

  public DriveCommand(DrivetrainSubsystem drivetrain, Axis forward, Axis strafe, Axis d) {
    this.forward = forward;
    this.strafe = strafe;
    this.rotation = d;

    drivetrainSubsystem = drivetrain;

    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    drivetrainSubsystem.drive(
        new Vector2(-forward.get(true) / 2, strafe.get(true) / 2), rotation.get(true) / 3);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.drive(Vector2.ZERO, 0, false);
  }
}
