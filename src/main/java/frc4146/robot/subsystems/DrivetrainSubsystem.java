package frc4146.robot.subsystems;

import static frc4146.robot.Constants.DriveConstants;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import common.control.*;
import common.drivers.Gyroscope;
import common.kinematics.ChassisVelocity;
import common.kinematics.SwerveKinematics;
import common.kinematics.SwerveOdometry;
import common.math.RigidTransform2;
import common.math.Rotation2;
import common.math.Vector2;
import common.robot.UpdateManager;
import common.util.*;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.*;

public class DrivetrainSubsystem implements Subsystem, UpdateManager.Updatable {
  public boolean driveFlag = true;

  //This value is used to turn the robot back to its initialPosition

  public ArrayList<Double> speeds;

  public static final double TRACKWIDTH = 24.0;
  public static final double WHEELBASE = 24.0;

  public Timer timer;

  public boolean fieldOriented;

  public static final DrivetrainFeedforwardConstants FEEDFORWARD_CONSTANTS =
      new DrivetrainFeedforwardConstants(
          0.70067, 2.2741, 0.16779);

  public static final TrajectoryConstraint[] TRAJECTORY_CONSTRAINTS = {
    new FeedforwardConstraint(
        11.0,
        FEEDFORWARD_CONSTANTS.getVelocityConstant(),
        FEEDFORWARD_CONSTANTS.getAccelerationConstant(),
        false),
    new MaxAccelerationConstraint(12.5), // originally 12.5 * 12.0
    new CentripetalAccelerationConstraint(15.0) // originally 15.0 * 12.0
  };

  private final HolonomicMotionProfiledTrajectoryFollower follower =
      new HolonomicMotionProfiledTrajectoryFollower(
          new PidConstants(0.035597, 0.0, 0.0015618),
          new PidConstants(0.035597, 0.0, 0.0015618),
          new HolonomicFeedforward(FEEDFORWARD_CONSTANTS));

  private final SwerveKinematics swerveKinematics =
      new SwerveKinematics(
          new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0), // front left
          new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0), // front right
          new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0), // back left
          new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0) // back right
          );

  private final SwerveModule[] modules;
  private final TalonSRX[] talons;

  private final Gyroscope gyroscope;

  private final SwerveOdometry swerveOdometry =
          new SwerveOdometry(swerveKinematics, RigidTransform2.ZERO);

  private RigidTransform2 pose = RigidTransform2.ZERO;
  private Vector2 velocity = Vector2.ZERO;
  private double angularVelocity = 0.0;

  private HolonomicDriveSignal driveSignal;

  // Logging
  private final GenericEntry odometryXEntry;
  private final GenericEntry odometryYEntry;
  private final GenericEntry odometryAngleEntry;


  public DrivetrainSubsystem(Gyroscope gyro) {

    gyroscope = gyro;

    gyroscope.setInverted(false);
    driveSignal = new HolonomicDriveSignal(new Vector2(0, 0), 0.0, true);

    timer = new Timer();

    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");



    SwerveModule frontLeftModule =
        Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                .withPosition(2, 0)
                .withSize(2, 4),
            Mk4SwerveModuleHelper.GearRatio.L2,
            DriveConstants.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR,
            DriveConstants.DRIVETRAIN_FRONT_LEFT_STEER_MOTOR,
            DriveConstants.DRIVETRAIN_FRONT_LEFT_STEER_ENCODER,
            DriveConstants.DRIVETRAIN_FRONT_LEFT_STEER_OFFSET);
    SwerveModule frontRightModule =
        Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                .withPosition(4, 0)
                .withSize(2, 4),
            Mk4SwerveModuleHelper.GearRatio.L2,
            DriveConstants.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR,
            DriveConstants.DRIVETRAIN_FRONT_RIGHT_STEER_MOTOR,
            DriveConstants.DRIVETRAIN_FRONT_RIGHT_STEER_ENCODER,
            DriveConstants.DRIVETRAIN_FRONT_RIGHT_STEER_OFFSET);
    SwerveModule backLeftModule =
        Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                .withPosition(6, 0)
                .withSize(2, 4),
            Mk4SwerveModuleHelper.GearRatio.L2,
            DriveConstants.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR,
            DriveConstants.DRIVETRAIN_BACK_LEFT_STEER_MOTOR,
            DriveConstants.DRIVETRAIN_BACK_LEFT_STEER_ENCODER,
            DriveConstants.DRIVETRAIN_BACK_LEFT_STEER_OFFSET);
    SwerveModule backRightModule =
        Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                .withPosition(8, 0)
                .withSize(2, 4),
            Mk4SwerveModuleHelper.GearRatio.L2,
            DriveConstants.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR,
            DriveConstants.DRIVETRAIN_BACK_RIGHT_STEER_MOTOR,
            DriveConstants.DRIVETRAIN_BACK_RIGHT_STEER_ENCODER,
            DriveConstants.DRIVETRAIN_BACK_RIGHT_STEER_OFFSET);

    modules =
        new SwerveModule[] {frontLeftModule, frontRightModule, backLeftModule, backRightModule};

    talons =
        new TalonSRX[] {
          new TalonSRX(DriveConstants.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR),
          new TalonSRX(DriveConstants.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR),
          new TalonSRX(DriveConstants.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR),
          new TalonSRX(DriveConstants.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR)
        };

    for (var talon : talons) {
      talon.configPeakCurrentLimit(30); // max. current (amps)
      talon.configPeakCurrentDuration(
          5); // # milliseconds after peak reached before regulation starts
      talon.configContinuousCurrentLimit(20); // continuous current (amps) after regulation
      talon.configOpenloopRamp(.5); // # seconds to reach peak throttle
    }

    odometryXEntry = tab.add("X", 0.0).withPosition(0, 0).withSize(1, 1).getEntry();
    odometryYEntry = tab.add("Y", 0.0).withPosition(0, 1).withSize(1, 1).getEntry();
    odometryAngleEntry = tab.add("Angle", 0.0).withPosition(0, 2).withSize(1, 1).getEntry();

    tab.addNumber(
            "Trajectory X",
            () -> {
              if (follower.getLastState() == null) {
                return 0.0;
              }
              return follower.getLastState().getPathState().getPosition().x;
            })
        .withPosition(1, 0)
        .withSize(1, 1);
    tab.addNumber(
            "Trajectory Y",
            () -> {
              if (follower.getLastState() == null) {
                return 0.0;
              }
              return follower.getLastState().getPathState().getPosition().y;
            })
        .withPosition(1, 1)
        .withSize(1, 1);

    tab.addNumber(
        "Rotation Voltage",
        () -> {
          HolonomicDriveSignal signal;
          signal = driveSignal;
          if (signal == null) {
            return 0.0;
          }

          return signal.getRotation() * RobotController.getBatteryVoltage();
        });

    tab.addNumber("Average Velocity", this::getAverageAbsoluteValueVelocity);
    tab.addBoolean("Drive Enabled",()-> driveFlag);
    tab.addBoolean("Field Oriented",()-> fieldOriented);
  }

  public void drive(Vector2 translationalVelocity, double rotationalVelocity, boolean isFieldOriented) {
    driveSignal = new HolonomicDriveSignal(translationalVelocity, rotationalVelocity, isFieldOriented);
  }

  public void drive(Vector2 translationalVelocity, double rotationalVelocity) {
    drive(translationalVelocity, rotationalVelocity, fieldOriented);
  }


  private void updateOdometry(double time, double dt) {
    Vector2[] moduleVelocities = getModuleVelocities();
    Rotation2 angle = gyroscope.getAngle();
    double angularVelocity = gyroscope.getRate();

    ChassisVelocity velocity = swerveKinematics.toChassisVelocity(moduleVelocities);

    this.pose = swerveOdometry.update(angle, dt, moduleVelocities);
    this.velocity = velocity.getTranslationalVelocity();
    this.angularVelocity = angularVelocity;
  }

  private void updateModules(HolonomicDriveSignal driveSignal, double dt) {
    Rotation2 rotOffset = (driveSignal.isFieldOriented()) ? getPose().rotation.inverse() : Rotation2.ZERO;
    ChassisVelocity chassisVelocity = new ChassisVelocity(driveSignal.getTranslation().rotateBy(rotOffset), driveSignal.getRotation());
    Vector2[] moduleOutputs = swerveKinematics.toModuleVelocities(chassisVelocity);
    SwerveKinematics.normalizeModuleVelocities(moduleOutputs, 1);
    for (int i = 0; i < moduleOutputs.length; i++) {
      modules[i].set(moduleOutputs[i].length * 12.0, moduleOutputs[i].getAngle().toRadians());
    }
  }

  @Override
  public void update(double time, double dt) {
    updateOdometry(time, dt);
    HolonomicDriveSignal driveSignal;
    Optional<HolonomicDriveSignal> trajectorySignal =
            follower.update(getPose(), getVelocity(), getAngularVelocity(), time, dt);
    driveSignal = trajectorySignal.orElseGet(() -> this.driveSignal);
    if(!driveFlag) {driveSignal = new HolonomicDriveSignal(Vector2.ZERO,0,false);}
    updateModules(driveSignal, dt);
  }

  @Override
  public void periodic() {
    updateDisplay();
  }

  public void updateDisplay() {
    RigidTransform2 pose = getPose();
    odometryXEntry.setDouble(pose.translation.x);
    odometryYEntry.setDouble(pose.translation.y);
    odometryAngleEntry.setDouble(pose.rotation.toDegrees());
  }

  public double getAverageAbsoluteValueVelocity() {
    return Arrays.stream(modules).mapToDouble(
            m -> Math.abs(m.getDriveVelocity()))
            .sum() / modules.length;
  }
  public Vector2[] getModuleVelocities() {
    return Arrays.stream(modules).map(m ->
            Vector2.fromAngle(Rotation2.fromRadians(m.getSteerAngle()))
              .scale(m.getDriveVelocity()*39.37008)).toArray(Vector2[]::new);
  }

  public RigidTransform2 getPose() {return pose;}
  public HolonomicDriveSignal getDriveSignal() {return driveSignal;}
  public HolonomicMotionProfiledTrajectoryFollower getFollower() {return follower;}

  public Vector2 getVelocity() {return velocity;}
  public double getAngularVelocity() {return angularVelocity;}

  public void toggleFieldOriented() {fieldOriented = !fieldOriented;}
  public void toggleDriveFlag() {driveFlag = !driveFlag;}

  public void resetPose(RigidTransform2 pose) {this.pose = pose; swerveOdometry.resetPose(pose);}
  public void resetGyroAngle(Rotation2 angle) {
    gyroscope.setAdjustmentAngle(gyroscope.getUnadjustedAngle().rotateBy(angle.inverse()));
  }
}
