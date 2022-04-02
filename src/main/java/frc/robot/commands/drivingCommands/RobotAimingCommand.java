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
    //this.limeLight.setPipeline();
    double steering_adjust = 0.0f;
    double turnCommand = 0;
    

    if (this.limeLight.getUpdatedTxValue() > 1.0) {
      steering_adjust = this.limeLight.getAimingValue() * (this.limeLight.getheading_error() - this.limeLight.getMin_Command());

    } else if (this.limeLight.getUpdatedTxValue() < 1.0) {
      steering_adjust = this.limeLight.getAimingValue() * this.limeLight.getheading_error() + this.limeLight.getMin_Command();
    }
   turnCommand += steering_adjust;
    turnCommand -= steering_adjust;

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    drive.arcadeDrive(0, turnCommand);
  }

  @Override
  public boolean isFinished() {
    return false; //robot.operatorInputs.getVisionButtonStatus();
  }
}
