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
  public static class OperatorConstants {
    // Port numbers for driver and operator gamepads. These correspond with the numbers on the USB
    // tab of the DriverStation
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static class DrivetrainConstants {
    // PWM ports/CAN IDs for motor controllers
    // public static final int kLeftRearID = 1;
    public static final int kLeftFrontID = 2;
    // public static final int kRightRearID = 3;
    public static final int kRightFrontID = 4;

    // Current limit for drivetrain motors
    // public static final int kCurrentLimit = 60;
  }

  public static class ShooterConstants {
    // PWM ports/CAN IDs for motor controllers
    public static final int kFeederID = 5;
    public static final int kShooterID = 6;

    // Current limit for shooter and feed wheels
    // public static final int kShooterCurrentLimit = 80;
    // public static final int kFeedCurrentLimit = 80;

    // Speeds for wheels when intaking and launching. Intake speeds are negative to run the wheels
    // in reverse
    public static final double kShooterSpeed = 1;
    public static final double kFeederSpeed = 1;
    public static final double kIntakeShooterSpeed = -1;
    public static final double kIntakeFeederSpeed = -.2;

    public static final double kShooterTimeDelay = 1.0;

    public static final double kShooterP = 0.00005;
    public static final double kShooterI = 0.0;
    public static final double kShooterD = 0.0;
    public static final double kShooterFF = 0.0002;
  }
}
