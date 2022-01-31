package frc.robot.constants;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public class DriveConstants {

    public static final double ksVolts = 0.046401;
    public static final double kvVoltSecondsPerMeter = 0.14137;
    public static final double kaVoltSecondsSquaredPerMeter = 0.0068514;
    public static final double kDriveTrackWidthMeters = 0.68;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kDriveTrackWidthMeters);
    public static final double kPDriveVel = 0.27454;
}
