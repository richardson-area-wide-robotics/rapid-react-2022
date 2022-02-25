package frc.robot.operatorInputs;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.BangBangArm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.drive.Drive;

public class OperatorInputs {

  private final double JOYSTICK_DEADZONE = 0.1;

  public OperatorInputs(Controls driverControls, Drive drive, BangBangArm bangArm, Intake intake) {

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
        .getJoystickXButton()
        .whenPressed(new InstantCommand(() -> bangArm.toggleArmPosition(), bangArm));

    driverControls.getJoystickBButton().whenPressed(new InstantCommand(() -> intake.gather(), intake));
    driverControls.getJoystickBButton().whenReleased(new InstantCommand(() -> intake.idle(), intake));
    driverControls.getJoystickAButton().whenPressed(new InstantCommand(() -> intake.outtake(), intake));
    driverControls.getJoystickAButton().whenReleased(new InstantCommand(() -> intake.idle(), intake));
  }
}
