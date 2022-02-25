package frc.robot.operatorInputs;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.BangBangArm;
import frc.robot.subsystems.Hangar;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.drive.Drive;

public class OperatorInputs {

  private final double JOYSTICK_DEADZONE = 0.1;

  public OperatorInputs(
      Controls driverControls, Drive drive, BangBangArm bangArm, Intake intake, Hangar hangar) {

    // Driver commands
    drive.setDefaultCommand(
        new RunCommand(
            () -> {
              // Invert getLeftY because the axis has 1 as down and -1 as up, while we want up = 1 =
              // forward.
              drive.curvatureDrive(
                  -driverControls.getLeftY(JOYSTICK_DEADZONE),
                  driverControls.getRightX(JOYSTICK_DEADZONE));
            },
            drive));

    driverControls
        .getJoystickXButton()
        .whenPressed(new InstantCommand(() -> bangArm.toggleArmPosition(), bangArm));

    driverControls
        .getJoystickBButton()
        .whenPressed(new InstantCommand(() -> intake.gather(), intake));
    driverControls
        .getJoystickBButton()
        .whenReleased(new InstantCommand(() -> intake.idle(), intake));
    driverControls
        .getJoystickAButton()
        .whenPressed(new InstantCommand(() -> intake.outtake(), intake));
    driverControls
        .getJoystickAButton()
        .whenReleased(new InstantCommand(() -> intake.idle(), intake));

    if (hangar.isAtZero()) {
      driverControls
          .getJoystickYButton()
          .whenPressed(new InstantCommand(() -> hangar.runToMidHeight(), hangar))
          .whenPressed(new InstantCommand(() -> hangar.engageMidHooks(), hangar));
    } else if (hangar.isAtMidHeight()) {
      driverControls
          .getJoystickYButton()
          .whenPressed(new InstantCommand(() -> hangar.runToReleaseHeight(), hangar));
    } else if (hangar.isAtReleaseHeight()) {
      driverControls
          .getJoystickYButton()
          .whenPressed(new InstantCommand(() -> hangar.releaseFlippyHooks(), hangar));
      driverControls
          .getJoystickYButton()
          .whenPressed(new InstantCommand(() -> hangar.releaseMidHooks(), hangar));
    }
  }

  // hanger controlls
  // raise hanger to mid hooks
  // engage mid hangers

  // when on the bar raise to flippy hooks release

  // when at flippy hooks release release the flippy hooks

  // when the flippy hooks are engaged release the mid hooks
}
