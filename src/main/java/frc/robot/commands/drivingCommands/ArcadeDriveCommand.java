package frc.robot.commands.drivingCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.operatorInputs.Controls;
import frc.robot.subsystems.drive.Drive;

public class ArcadeDriveCommand extends CommandBase {

    private Drive drive;
    private Controls controls;
    private double DEADZONE = 0.2;
  
    public ArcadeDriveCommand(Controls controls, Drive drive) {
        this.controls = controls;
        this.drive = drive;
        addRequirements(drive);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        drive.arcadeDrive(controls.getLeftY(DEADZONE), controls.getRightX(DEADZONE));
    }
}