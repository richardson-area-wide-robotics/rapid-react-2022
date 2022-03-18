package frc.robot.commands.drivingCommands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.operatorInputs.OperatorInputs;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.LimeLight;

public class RobotAimingCommand extends CommandBase {
  public Drive drive;
  public Robot robot;
  public LimeLight limeLight;
  public OperatorInputs operatorInputs;

  // Called just before this Command runs the first time
  @Override
  public void initialize() {}

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    double steeringAdjust = limeLight.getrobotAiming(1);
    drive.arcadeDrive(0, steeringAdjust);
  }

  @Override
  public boolean isFinished() {
    return robot.operatorInputs.getVisionButtonStatus();
  }
}
