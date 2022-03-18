package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyroscope extends SubsystemBase {

  private AHRS ahrs;
  // variables
  private double startTime;
  private double driftPerSecond;
  private Rotation2d lastHeading = Rotation2d.fromDegrees(0);

  public Gyroscope() {
    this.ahrs = new AHRS(SPI.Port.kMXP);
    this.calibrate();
    this.register();
  }

  public double getGyroAngle() {
    double runTime = Timer.getFPGATimestamp() - startTime;
    double drift = runTime * driftPerSecond;
    return this.ahrs.getAngle();
  }

  public Rotation2d getRotation2d() {
    if (ahrs.isConnected()) {
      return this.ahrs.getRotation2d();
    } else {
      return lastHeading;
    }
  }

  public double getRate() {
    return this.ahrs.getRate();
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
    lastHeading = Rotation2d.fromDegrees(0);
  }

  public boolean isConnected(){
    return ahrs.isConnected();
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

  @Override
  public void periodic() {
    lastHeading = getRotation2d();
  }

  /**
   * Informs the gyro of a rotation seen via wheel odometry. Only impacts last known heading, in
   * case of gyro dropout
   *
   * @param offset
   */
  public void rotationFromWheelOdometry(Rotation2d offset) {
    // Add manually by degrees since the methods clamp to (0, 360)
    lastHeading = Rotation2d.fromDegrees(lastHeading.getDegrees() + offset.getDegrees());
  }
}
