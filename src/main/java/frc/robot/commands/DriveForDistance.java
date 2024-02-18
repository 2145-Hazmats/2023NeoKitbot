// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDrivetrain;

public class DriveForDistance extends Command {
  /** Creates a new DriveForDistance. */ 
  
  CANDrivetrain drivetrain;
  
  double error;
  double Setpoint; 
  double kp = .28;
  double SPEED;
  
  
  public DriveForDistance(CANDrivetrain drie, double setpoint) {
    drivetrain = drie;
    Setpoint = setpoint;
    addRequirements(drie);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   drivetrain.ResetEncoders();
  

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   drivetrain.FeedDrivetrain();
   error =  Setpoint -  SmartDashboard.getNumber("Average Drive Motor Encoder Value in Feets", 0);
   SPEED = kp * error;
   drivetrain.driveForward(SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //drivetrain.driveForward(-0.121733088046312);
   // drivetrain.ResetEncoders();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (drivetrain.distance >= 10);
    
  }
}
