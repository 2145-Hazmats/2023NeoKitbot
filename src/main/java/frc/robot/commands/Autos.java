// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.CANDrivetrain;
import frc.robot.subsystems.Shooter;

public final class Autos {

  public static Command ShootAndDrive(CANDrivetrain drivetrain, Shooter shooter) {
    drivetrain.setDrivetrainSafety(false);
    return Commands.sequence(
      //Commands.repeatingSequence(new RunCommand(() -> drivetrain.FeedDrivetrain(), drivetrain)),
      Commands.parallel( 
        //drivetrain.arcadeDrive(0, 0),
        //new RunCommand(() -> drivetrain.arcadeDrive(1, 0))/*.withTimeout(5),
        new RunCommand(() -> shooter.setLaunchWheel(1.0), shooter),
        new RunCommand(() -> drivetrain.driveForward(0.25), drivetrain)
        .withTimeout(1.0)
        .andThen(new RunCommand(() -> drivetrain.driveForward(0.5), drivetrain))
        .andThen(new RunCommand(() -> drivetrain.driveForward(0.0), drivetrain))
        .withTimeout(0.5)
        //new RunCommand(() -> { (drivetrain.driveForwardCommand(0.25), drivetrain); }).withTimeout(1.0)
        //drivetrain.driveForwardCommand(0.6)) 
      ).withTimeout(2.0),
      new RunCommand(() -> shooter.setFeedWheel(1), shooter).withTimeout(1.0),
      new RunCommand(() -> drivetrain.driveForward(-0.6), drivetrain).withTimeout(1),
      new RunCommand(() -> drivetrain.driveForward(0.0), drivetrain).withTimeout(0.5)
    );
  }
    
}
 
//public final class Autos {
  /** Example static factory for an autonomous command. 
  public static Command exampleAuto(CANDrivetrain drivetrain) {
    /**
     * RunCommand is a helper class that creates a command from a single method, in this case we
     * pass it the arcadeDrive method to drive straight back at half power. We modify that command
     * with the .withTimeout(1) decorator to timeout after 1 second, and use the .andThen decorator
     * to stop the drivetrain after the first command times out
     
    return new RunCommand(() -> drivetrain.arcadeDrive(-.5, 0))
        .withTimeout(1)
        .andThen(new RunCommand(() -> drivetrain.arcadeDrive(0, .5)))
        .withTimeout(1);
        
  }

}*/