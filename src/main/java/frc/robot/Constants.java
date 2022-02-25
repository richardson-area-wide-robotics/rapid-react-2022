// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double ARM_SPROCKET_RATIO = 16.0 / 64.0; // 64-tooth to 16-tooth reduction
  public static final double ARM_PLANETARY_RATIO = 1.0 / 36.0; // 36 to 1 reduction
  public static final double NEO_REVOLUTIONS_TO_DEGREES =
      1.0; // conversion from revolutions to degrees for a neo motor
  public static final double ARM_POSITION_CONVERISON_FACTOR =
      (ARM_SPROCKET_RATIO * ARM_PLANETARY_RATIO * NEO_REVOLUTIONS_TO_DEGREES);
}
