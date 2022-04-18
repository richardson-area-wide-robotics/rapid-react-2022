package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight extends SubsystemBase {
  // public Drive drive;
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry pipeline = table.getEntry("getpipe");
  NetworkTableEntry ts = table.getEntry("ts");
  NetworkTableEntry camtran = table.getEntry("camtran");

  public void setPipeline() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
  }

  // read values periodically
  double x = tx.getDouble(0.0);
  double y = ty.getDouble(0.0);
  double v = tv.getDouble(0.0);
  double area = ta.getDouble(0.0);

  // values for distance
  double targetOffsetAngle_Vertical = y;

  // how many degrees back is your limelight rotated from perfectly vertical?
  double limelightMountAngleDegrees = 0; // this value needs to be figured out

  // distance from the center of the Limelight lens to the floor
  double limelightLensHeightInches = 0; // this value also needs to be figured out

  // distance from the target to the floor
  double goalHeightInches = 0; // this value also needs to be figured out

  double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
  double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

  // calculate distance
  double distanceFromLimelightToGoalInches =
      (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

  // desired distance
  double desired_distance = 0.0; // number here should be in inches

  // Values for turning robot to target - need to adjust them for perfect turning
  public double Kp = 0.05; // neeed to figure out this value for perfect aiming to target
  double min_Command = 0.15; // also need to figure out this value
  double KpDistance = -0.1f; // Proportional control constant for distance
  double KpAim = -0.1f;
  double current_distance =
      distanceFromLimelightToGoalInches; // see the 'Case Study: Estimating Distance'
  float min_aim_command = 0.1f;
  double heading_error = -getUpdatedTxValue();

  public LimeLight() {
    // post to smart dashboard periodically
  }

  public void smartDashboard() {
    // values i Want to see for smartdashboard
    SmartDashboard.putNumber("LimelightX", getUpdatedTxValue());
    // SmartDashboard.putNumber("LimelightY", getUpdatedTyValue());
    // SmartDashboard.putNumber("LimelightV", getUpdatedTvValue());
    // SmartDashboard.putNumber("LimelightArea", area);
  }

  public double getMin_Command() {
    return min_Command;
  }

  public boolean getIsTargetFound() {
    NetworkTableEntry tv = table.getEntry("tv");
    double v = tv.getDouble(0);
    if (v == 0.0f) {
      return false;
    } else {
      return true;
    }
  }

  public double getUpdatedTvValue() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0);
  }

  public double getUpdatedTxValue() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
  }

  public double getUpdatedTyValue() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
  }

  public double getheading_error() {
    return heading_error;
  }

  public double getTxValue() {
    return x;
  }

  public double getAimingValue() {
    return Kp;
  }

  public boolean greaterTxAngle() {
    return getUpdatedTxValue() > 1.0;
  }

  public boolean lessTxAngle() {
    return getUpdatedTxValue() < 1.0;
  }

  public double getrobotAiming(double turn) {
    double steering_adjust = 0.0f;
    double heading_error = x * -1;
    if (x > 1.0) {

      steering_adjust = Kp * (heading_error - min_Command);
    } else if (x < 1.0) {
      steering_adjust = Kp * heading_error + min_Command;
    }
    turn += steering_adjust;
    turn -= steering_adjust;

    return steering_adjust;
  }

  public void seekTarget() {
    double steering_adjust = 0.0f;
    if (v == 0.0f) {
      // We don't see the target, seek for the target by spinning in place at a safe speed.
      steering_adjust = 0.3f;
    } else {
      // We do see the target, execute aiming code
      double heading_error = x;
      steering_adjust = Kp * x;
    }
    // left_command+=steering_adjust;
    // right_command-=steering_adjust;
  }

  public void robotInRange() {
    double distance_error = desired_distance - current_distance;
    double driving_adjust = KpDistance * distance_error;

    // left_command += distance_adjust;
    // right_command += distance_adjust;
  }

  public boolean hasValidTarget() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0)
        > 0;
  }

  public void aimAndMoveRobot() {
    double steering_adjust = 0.0f;
    double heading_error = -x;
    double distance_error = -y;

    if (x > 1.0) {
      steering_adjust = KpAim * heading_error - min_aim_command;
    } else if (x < -1.0) {
      steering_adjust = KpAim * heading_error + min_aim_command;
    }

    double distance_adjust = KpDistance * distance_error;

    // left_command += steering_adjust + distance_adjust;
    // right_command -= steering_adjust + distance_adjust;
  }
}
