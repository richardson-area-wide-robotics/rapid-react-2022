package frc.robot.constants;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public class DriveConstants {

  public static final double ksVolts = 0.23603; //0.22243;
  public static final double kvVoltSecondsPerMeter = 2.5615; //5.912;
  public static final double kaVoltSecondsSquaredPerMeter = 3.7538; //0.10605;
  public static final double kDriveTrackWidthMeters = 0.68;
  public static final DifferentialDriveKinematics kDriveKinematics =
      new DifferentialDriveKinematics(kDriveTrackWidthMeters);
  public static final double kPDriveVel = 3.9805; //2.0903;
  public static final double WHEEL_DIAMETER_METERS = 0.1016;
}
