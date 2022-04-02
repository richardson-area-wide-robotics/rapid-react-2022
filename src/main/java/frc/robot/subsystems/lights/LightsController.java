package frc.robot.subsystems.lights;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.sensors.TOFSensor;
import frc.robot.subsystems.vision.LimeLight;

public class LightsController extends SubsystemBase {
  private Lights lights;
  private TOFSensor tofSensor;
 // private LimeLight limeLight;
  boolean isIdle = true;

  public LightsController(Lights lights, TOFSensor tofSensor/*, LimeLight limeLight*/) {
    this.lights = lights;
    this.tofSensor = new TOFSensor(1);
    //this.limeLight = limeLight;
    this.lights.idleAnimation(3);
    ;
  }

  /*public void checkTargetLock() {
    if (limeLight.hasValidTarget() && !isIdle) {
      lights.allLimeGreen();
      isIdle = true;
    } else if (!limeLight.hasValidTarget() && isIdle) {
      lights.idleAnimation(3);
      isIdle = false;
    }
  }*/
}
