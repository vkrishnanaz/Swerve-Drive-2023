package frc4146.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4146.robot.subsystems.DrivetrainSubsystem;
import common.control.Trajectory;
import edu.wpi.first.wpilibj.Timer;

public class FollowTrajectoryCommand extends CommandBase {
  private final DrivetrainSubsystem drivetrain;
  private final Trajectory trajectory;
  private final Timer timerElapsed;
  private final Timer timer;

  public FollowTrajectoryCommand(DrivetrainSubsystem drivetrain, Trajectory trajectory) {
    this.drivetrain = drivetrain;
    this.trajectory = trajectory;
    this.timerElapsed = new Timer();
    this.timer = new Timer();

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    drivetrain.getFollower().follow(trajectory);
    timer.reset();
    timerElapsed.reset();
  }
  @Override
  public void execute() {
    drivetrain.update(timerElapsed.get(), timer.get());
    timer.reset();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.getFollower().cancel();
  }

  @Override
  public boolean isFinished() {
    return drivetrain.getFollower().getCurrentTrajectory().isEmpty();
  }
}
