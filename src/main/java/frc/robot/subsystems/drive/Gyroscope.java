package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;

public class Gyroscope {

  private AHRS ahrs;
  // variables
  private double startTime;
  private double driftPerSecond;

  public Gyroscope() {
    this.ahrs = new AHRS(SPI.Port.kMXP);
    this.calibrate();
  }

  public double getGyroAngle() {
    double runTime = Timer.getFPGATimestamp() - startTime;
    double drift = runTime * driftPerSecond;
    return this.ahrs.getAngle();
  }

  public Rotation2d getRotation2d() {
    return this.ahrs.getRotation2d();
  }

  public void getRate() {
    this.ahrs.getRate();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return this.ahrs.getRotation2d().getDegrees();
  }

  public void resetGyro() {
    this.ahrs.reset();
    this.startTime = Timer.getFPGATimestamp();
  }

  public void calibrate() {
    this.startTime = Timer.getFPGATimestamp();
    double startAngle = ahrs.getAngle();
    try {
      Thread.sleep(1000);
    } catch (Exception e) {
    }
    this.driftPerSecond = (ahrs.getAngle() - startAngle) / (Timer.getFPGATimestamp() - startTime);
  }
}
