package frc.robot.constants;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public class DriveConstants {

    public static final double ksVolts = 0.22243;
    public static final double kvVoltSecondsPerMeter = 5.912;
    public static final double kaVoltSecondsSquaredPerMeter = 0.10605;
    public static final double kDriveTrackWidthMeters = 0.68;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kDriveTrackWidthMeters);
    public static final double kPDriveVel = 2.0903;
}
