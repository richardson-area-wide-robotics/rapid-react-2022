package frc.robot.operatorInputs;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.drive.Drive;



public class OperatorInputs {

  private final double JOYSTICK_DEADZONE = 0.1;

  public OperatorInputs(Controls driverControls, Drive drive) {

    // Driver commands
    drive.setDefaultCommand(new RunCommand(() -> {
      // Invert getLeftY because the axis has 1 as down and -1 as up, while we want up = 1 = forward.
      drive.curvatureDrive(-driverControls.getLeftY(JOYSTICK_DEADZONE), driverControls.getRightX(JOYSTICK_DEADZONE));
    }, drive));
  }
}
