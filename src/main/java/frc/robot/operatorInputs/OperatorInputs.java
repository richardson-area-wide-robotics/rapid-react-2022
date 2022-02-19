package frc.robot.operatorInputs;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.drive.Drive;

public class OperatorInputs {

  private final double JOYSTICK_DEADZONE = 0.1;

  public OperatorInputs(Controls driverControls, Drive drive, Arm arm) {

    // Driver commands
    drive.setDefaultCommand(
        new RunCommand(
            () -> {
              drive.curvatureDrive(
                  driverControls.getLeftY(JOYSTICK_DEADZONE),
                  driverControls.getRightX(JOYSTICK_DEADZONE));
            },
            drive));

    driverControls
        .getJoystickYButton()
        .whenPressed(new InstantCommand(() -> arm.toggleArmPosition(), arm));
  }
}
