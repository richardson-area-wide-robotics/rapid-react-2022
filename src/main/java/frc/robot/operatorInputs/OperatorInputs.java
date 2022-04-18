package frc.robot.operatorInputs;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.arm.BangBangArm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hangar.Trav;

public class OperatorInputs {

  private final double JOYSTICK_DEADZONE = 0.1;

  public OperatorInputs(
      Controls driverControls,
      Controls operatorControls,
      Drive drive,
      BangBangArm bangArm,
      Intake intake,
      Trav trav
      /*RobotAimingCommand aimRobot*/ ) {

    // Driver commands
    drive.setDefaultCommand(
        new RunCommand(
            () -> {
              // Invert getLeftY because the axis has 1 as down and -1 as up, while we want up = 1 =
              // forward.
              double throttle = -driverControls.getLeftY(JOYSTICK_DEADZONE);
              double turn = driverControls.getRightX(JOYSTICK_DEADZONE);
              turn *= 1.0 * Math.abs(turn * turn);
              drive.curvatureDrive(throttle, turn);
            },
            drive));
    // trav.setDefaultCommand(
    //     new  RunCommand(
    //         () -> {
    //             if (operatorControls.getLeftY(JOYSTICK_DEADZONE) >= 0.15){
    //                 new InstantCommand(() -> trav.manualMoveArmIn(), trav);
    //                 SmartDashboard.putString("arm Up", "if the arm is up");
    //             }
    //            else if (operatorControls.getLeftY(JOYSTICK_DEADZONE) <= -0.15){
    //             new InstantCommand(() -> trav.manualMoveArmOut(), trav);
    //             SmartDashboard.putString("arm Down", "if the arm is Down");
    //            }
    //         }, trav)
    // );

    // driverControls.getLeftJoystickBumper().whenPressed(aimRobot, operatorControls.getAButton());
    operatorControls
        .getJoystickYButton()
        .whenPressed(new InstantCommand(() -> trav.runToMidHeight(), trav));
    operatorControls
        .getJoystickBButton()
        .whenPressed(new InstantCommand(() -> trav.manualMoveArmOut()));
    operatorControls
        .getJoystickAButton()
        .whenPressed(new InstantCommand(() -> trav.manualMoveArmIn(), trav));
    driverControls
        .getJoystickYButton()
        .whenPressed(
            new SequentialCommandGroup(
                new InstantCommand(() -> trav.runToArmOutLineUp()),
                new WaitCommand(.5),
                new InstantCommand(
                    () ->
                        trav
                            .runToHookHeight()) /*, new WaitCommand(1.0), new InstantCommand(() -> runToArmReleaseHeight())*/)); // hangar.runToLineUp(), hangar // hangar.runToHookHeight(), hangar
    // operatorControls.getJoystickLeftTrigger().whenActive(new InstantCommand(() ->
    // trav.manualMoveArmIn(), trav));
    // operatorControls.getJoystickRightTrigger().whenActive(new InstantCommand(() ->
    // trav.manualMoveArmOut(), trav));

    //     if (operatorControls.getPOV() > 350 && operatorControls.getPOV() < 10 ){
    //         new InstantCommand(() -> trav.manualMoveArmIn(), trav);
    //         SmartDashboard.putString("arm Up", "if the arm is up");
    //     }
    //    else if (operatorControls.getPOV() > 170 && operatorControls.getPOV() < 190){
    //     new InstantCommand(() -> trav.manualMoveArmOut(), trav);
    //     SmartDashboard.putString("arm Down", "if the arm is Down");
    //    }
    //    SmartDashboard.putNumber("up dpad", operatorControls.getPOV());
    //    SmartDashboard.putNumber("down dpad", operatorControls.getPOV());

    operatorControls
        .getJoystickXButton()
        .whenPressed(new InstantCommand(() -> bangArm.toggleArmPosition(), bangArm));

    // operatorControls
    //     .getJoystickXButton()
    //     .whenPressed(new InstantCommand(() -> bangArm.armPositioningManipulation(), bangArm));

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

  }

  public boolean getVisionButtonStatus() {
    return false;
  }
  // hanger controlls
  // raise hanger to mid hooks
  // engage mid hangers

  // when on the bar raise to flippy hooks release

  // when at flippy hooks release release the flippy hooks

  // when the flippy hooks are engaged release the mid hooks
}
