// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double deadzone = 0.05;
  }

  public class SwerveConstants{
    public static final double maxSpeed = (8);
    
  }

  public static class PivotConstants {
    public static final double PivotUpperBoundLimit = 99;
    public static final double PivotLowerBoundLimit = 99;
    public static final double PivotMaxVelocity = 99;
  public static class IntakeConstants {
    public static final int IntakeTalonCurrentLimit = 40;
    public static final int IntakeTalonVoltage = 6;
  }

  public static class ShooterConstants {
    public static final double ShooterMaxRotations = 100;
    public static final double ShooterMinRotations = 0;
  }
}
