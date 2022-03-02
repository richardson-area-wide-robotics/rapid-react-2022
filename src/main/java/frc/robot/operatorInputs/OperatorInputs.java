package frc.robot.operatorInputs;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.arm.BangBangArm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hangar.Hangar;

public class OperatorInputs {

  private final double JOYSTICK_DEADZONE = 0.1;

  public OperatorInputs(
      Controls driverControls,
      Controls operatorControls,
      Drive drive,
      BangBangArm bangArm,
      Intake intake,
      Hangar hangar) {

    // Driver commands
    drive.setDefaultCommand(
        new RunCommand(
            () -> {
              // Invert getLeftY because the axis has 1 as down and -1 as up, while we want up = 1 =
              // forward.
              double throttle = -driverControls.getLeftY(JOYSTICK_DEADZONE);
              double turn = driverControls.getRightX(JOYSTICK_DEADZONE);
              turn *= Math.abs(turn);
              drive.curvatureDrive(throttle, turn);
            },
            drive));

    operatorControls
        .getJoystickXButton()
        .whenPressed(new InstantCommand(() -> bangArm.toggleArmPosition(), bangArm));

    operatorControls
        .getLeftJoystickBumper()
        .whenPressed(new InstantCommand(() -> intake.gather(), intake));
    operatorControls
        .getLeftJoystickBumper()
        .whenReleased(new InstantCommand(() -> intake.idle(), intake));
    operatorControls
        .getRightJoystickBumper()
        .whenPressed(new InstantCommand(() -> intake.outtake(), intake));
    operatorControls
        .getRightJoystickBumper()
        .whenReleased(new InstantCommand(() -> intake.idle(), intake));
    /*operatorControls
        .getJoystickBButton()
        .whenPressed(new InstantCommand(() -> hangar.runToReleaseHeight(), hangar));
    operatorControls
        .getJoystickAButton()
        .whenPressed(new InstantCommand(() -> hangar.releaseFlippyHooks(), hangar));
    driverControls
        .getJoystickBButton()
        .whenPressed(new InstantCommand(() -> hangar.engageMidHooks(), hangar));*/

    if (hangar.isAtZero()) {
      operatorControls
          .getJoystickYButton()
          .whenPressed(new InstantCommand(() -> hangar.runToMidHeight(), hangar))
          .whenPressed(new InstantCommand(() -> hangar.releaseMidHooks(), hangar));
    } /*else if (hangar.isAtMidHeight()) {
        operatorControls
            .getJoystickYButton()
            .whenPressed(new InstantCommand(() -> hangar.runToReleaseHeight(), hangar));
      } else if (hangar.isAtReleaseHeight()) {
        operatorControls
            .getJoystickYButton()
            .whenPressed(new InstantCommand(() -> hangar.releaseFlippyHooks(), hangar));
        operatorControls
            .getJoystickYButton()
            .whenPressed(new InstantCommand(() -> hangar.engageMidHooks(), hangar));
      }*/
  }

  // hanger controlls
  // raise hanger to mid hooks
  // engage mid hangers

  // when on the bar raise to flippy hooks release

  // when at flippy hooks release release the flippy hooks

  // when the flippy hooks are engaged release the mid hooks
}
