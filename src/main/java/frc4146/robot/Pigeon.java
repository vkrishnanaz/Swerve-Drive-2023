package frc4146.robot;

import com.ctre.phoenix.sensors.Pigeon2;

import common.drivers.Gyroscope;
import common.math.Rotation2;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class Pigeon extends Gyroscope {
  private final Pigeon2 pigeon;

  public Pigeon(int id) {
    this.pigeon = new Pigeon2(id);
    Shuffleboard.getTab("Drivetrain").addNumber("Pigeon Offset",()->getAdjustmentAngle().toDegrees());
  }

  @Override
  public void calibrate() {setAdjustmentAngle(getUnadjustedAngle());}

  @Override
  public Rotation2 getUnadjustedAngle() {
    return Rotation2.fromDegrees(pigeon.getYaw());
  }

  @Override
  public double getUnadjustedRate() {
    return 0.0;
  }
}
