// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

// import static frc.robot.Constants.ShooterConstants;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Shooter extends SubsystemBase {
  /** Creates a new PIDLauncher. */
  private CANSparkMax m_shooterMotor;

  private WPI_TalonSRX m_feedMotor;
  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder;
  private double kP, kI, kD, kFF, desiredRPM;
  private double kS, kV, kA;

  // why start variables up here and then give them value down there?

  public Shooter() {
    m_shooterMotor = new CANSparkMax(6, MotorType.kBrushless);
    m_encoder = m_shooterMotor.getEncoder();
    m_feedMotor = new WPI_TalonSRX(5);

    m_shooterMotor.restoreFactoryDefaults();
    m_feedMotor.configFactoryDefault(); // added this because I love factories

    m_shooterMotor.enableVoltageCompensation(11.5);

    m_pidController = m_shooterMotor.getPIDController();

    kP = 0.00005;
    kI = 0.000001;
    kD = 0.0001;
    kFF = 0.000156;
    desiredRPM = 5500;

    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setFF(kFF);

    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Set Velocity", desiredRPM);
  }

  // Intake command
  public Command intakeCommand() {
    return this.startEnd(
        () -> {
          m_feedMotor.set(ShooterConstants.kIntakeFeederSpeed);
          m_shooterMotor.set(ShooterConstants.kIntakeShooterSpeed);
        },
        () -> {
          this.stop();
        });
  }

  // Rotates the shooter motor
  public Command prepareNoteCommand() {
    return this.run(
        () -> {
          m_pidController.setReference(desiredRPM, CANSparkMax.ControlType.kVelocity);
        });
  }

  // Rotates both the feeder and shooter motor and then stops both
  public Command shootNoteCommand() {
    return this.startEnd(
        () -> {
          m_feedMotor.set(1.0);
          m_pidController.setReference(desiredRPM, CANSparkMax.ControlType.kVelocity);
        },
        () -> {
          stop();
        });
  }

  public void setLaunchWheel(double speed) {
    m_shooterMotor.set(speed);
  }

  public void setFeedWheel(double speed) {
    m_feedMotor.set(speed);
  }

  public void stop() {
    m_pidController.setReference(
        0,
        CANSparkMax.ControlType.kVelocity); // why do you need this alonside the motor stop stuff?
    m_shooterMotor.set(0);
    m_feedMotor.set(0);
  }

  public Command stopCommand() {
    return this.runOnce(
        () -> {
          stop();
        });
  }

  @Override
  public void periodic() {
    // Get PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0); 
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double moddedRPM = SmartDashboard.getNumber("Set Velocity", 0);

    // If PID coefficients on SmartDashboard have changed, write new values to controller
    if ((p != kP)) {
      m_pidController.setP(p);
      kP = p;
    } // != means not equal to
    if ((i != kI)) {
      m_pidController.setI(i);
      kI = i;
    }
    if ((d != kD)) {
      m_pidController.setD(d);
      kD = d;
    }
    if ((ff != kFF)) {
      m_pidController.setFF(ff);
      kFF = ff;
    }
    if ((moddedRPM != desiredRPM)) {
      desiredRPM = moddedRPM;
    }

    SmartDashboard.putNumber("ShooterVelocity", m_encoder.getVelocity());
    SmartDashboard.putNumber("ShooterVoltage", m_shooterMotor.getBusVoltage());
    SmartDashboard.putNumber("FeederSpeed", m_feedMotor.get());
  }
}
