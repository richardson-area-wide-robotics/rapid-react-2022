package frc.robot.operatorInputs;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Robot;
import frc.robot.commands.drivingCommands.CargoAimCommand;
import frc.robot.subsystems.drive.Drive;



public class OperatorInputs {

  private final double JOYSTICK_DEADZONE = 0.1;

  public OperatorInputs(Controls driverControls, Drive drive, CargoAimCommand cargoAimCommand) {

    driverControls.getRightJoystickBumper().whenHeld(cargoAimCommand);

    // Driver commands
    drive.setDefaultCommand(new RunCommand(() -> {
      drive.curvatureDrive(driverControls.getLeftY(JOYSTICK_DEADZONE), driverControls.getRightX(JOYSTICK_DEADZONE));
    }, drive));
  }
}
