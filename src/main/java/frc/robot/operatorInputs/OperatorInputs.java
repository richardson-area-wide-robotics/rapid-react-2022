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

    //  driverControls
    //      .getJoystickYButton()
    //      .whenPressed(new InstantCommand(() -> arm.toggleArmPosition(), arm));

    driverControls
        .getJoystickXButton()
        .whenPressed(new InstantCommand(() -> bangArm.toggleArmPosition(), bangArm));
        if (bangArm.atReverseLimit()){
          intake.gather();
        }

    driverControls.getJoystickBButton().whenActive(new InstantCommand(() -> intake.gather(), intake));
    driverControls.getJoystickAButton().whenActive(new InstantCommand(() -> intake.outtake(), intake));
    //driverControls.getJoystickYButton().whenPressed(new InstantCommand(() -> intake.idle(), intake));
    if (driverControls.getJoystickAButton().get() == false){
      intake.idle();
    }
  }
}
