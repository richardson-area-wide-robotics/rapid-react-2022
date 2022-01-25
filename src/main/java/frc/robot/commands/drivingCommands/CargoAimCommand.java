package frc.robot.commands.drivingCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.operatorInputs.Controls;
import frc.robot.subsystems.drive.Drive;

public class CargoAimCommand extends CommandBase{
    
    private Drive drive;
    private Controls controls;
    
    
    public CargoAimCommand(Drive drive, Controls controls) {
        this.drive = drive;
        this.controls = controls;
        addRequirements(drive);
        System.out.println("Testing command");
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        
    }
}
