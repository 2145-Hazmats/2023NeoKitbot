// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
//import frc.robot.commands.LimitSwitchFunction;

// import static frc.robot.Constants.ShooterConstants;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Shooter extends SubsystemBase {
  /** Creates a new PIDLauncher. */
  private WPI_TalonSRX m_leftShooterMotor;
  private WPI_TalonSRX m_rightShooterMotor;

  private WPI_TalonSRX m_leftFeedMotor;
  private WPI_TalonSRX m_rightFeedMotor;
  //private SparkMaxPIDController m_pidController;
  //private RelativeEncoder m_encoder;
  //private double kP, kI, kD, kFF, desiredRPM;
  //private double kS, kV, kA;
  public boolean redLimitSwitch;
  public boolean blackLimitSwitch;


  // why start variables up here and then give them value down there?

  public Shooter() {
    //m_shooterMotor = new CANSparkMax(ShooterConstants.kShooterID, MotorType.kBrushless);
    m_leftShooterMotor = new WPI_TalonSRX(ShooterConstants.kleftShooterID);
    m_rightShooterMotor = new WPI_TalonSRX(ShooterConstants.krightShooterID);
    m_leftFeedMotor = new WPI_TalonSRX(ShooterConstants.kleftFeederID);
    m_rightFeedMotor = new WPI_TalonSRX(ShooterConstants.krightFeederID);
    
    //m_encoder = m_shooterMotor.getEncoder();
    m_leftFeedMotor.configFactoryDefault(); // added this because I love factories
    m_rightFeedMotor.configFactoryDefault();
    m_leftShooterMotor.configFactoryDefault();
    m_rightShooterMotor.configFactoryDefault();
 
    //motor controller group thing
    //m_rightFeedMotor.follow(m_leftFeedMotor);
    //m_rightShooterMotor.follow(m_leftShooterMotor);
    
    //mirrors master motor
    m_rightFeedMotor.setInverted(true);
    m_rightShooterMotor.setInverted(true);


    //m_leftShooterMotor.enableVoltageCompensation(11.5);

    /*
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
  */
  }

  // Intake command
  public Command intakeCommand() {
    return this.startEnd(
        () -> {
          m_leftFeedMotor.set(ShooterConstants.kIntakeFeederSpeed);
          m_leftShooterMotor.set(ShooterConstants.kIntakeShooterSpeed);
          m_rightFeedMotor.set(ShooterConstants.kIntakeFeederSpeed);
          m_rightShooterMotor.set(ShooterConstants.kIntakeShooterSpeed);
        },
        () -> {
          this.stop();
        });
  }
  

  /*public Command limitSwitchIntakeCommand() {
    return this.startEnd(
        () -> {
          if (!m_limitswitch.get()) { // or two equal signs = false
          m_feedMotor.set(ShooterConstants.kIntakeFeederSpeed);
          m_shooterMotor.set(ShooterConstants.kIntakeShooterSpeed);
          }
        },
        () -> {
          this.stop();
        });
  }*/


  // Rotates the shooter motor
  /*
  public Command prepareNoteCommand() {
    return this.run(
        () -> {
         // m_pidController.setReference(desiredRPM, CANSparkMax.ControlType.kVelocity);
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
  */

  public boolean isRedSwitchPressed() {
   return  (redLimitSwitch);
  }

  public boolean isBlackSwitchPressed() {
    return  (blackLimitSwitch);
  }


  public void setLaunchWheel(double speed) {
    m_leftShooterMotor.set(speed);
    m_rightShooterMotor.set(speed);
  }

  public void setFeedWheel(double speed) {
    m_leftFeedMotor.set(speed);
    m_rightFeedMotor.set(speed);
  }

  public void Intake(double feedspeed, double shootspeed) {
    m_leftFeedMotor.set(feedspeed);
    m_leftShooterMotor.set(shootspeed);
    m_rightFeedMotor.set(feedspeed);
    m_rightShooterMotor.set(shootspeed);
  }
public Command readyAmpCommand() {
    return this.startEnd(
        () -> {
    m_leftShooterMotor.set(Constants.ShooterConstants.kAmpLeftShooter);
    m_rightShooterMotor.set(Constants.ShooterConstants.kAmpRightShooter);
        },
        () -> {
          this.stop();
        });
  }
  public Command playAmpCommand() {
    return this.startEnd(
        () -> {
    m_leftShooterMotor.set(Constants.ShooterConstants.kAmpLeftShooter);
    m_rightShooterMotor.set(Constants.ShooterConstants.kAmpRightShooter);
    m_leftFeedMotor.set(Constants.ShooterConstants.kAmpLeftFeeder);
    m_rightFeedMotor.set(Constants.ShooterConstants.kAmpRightFeeder);
        },
        () -> {
          this.stop();
        });
  }
  

  public void stop() {
    m_leftShooterMotor.set(0);
    m_leftFeedMotor.set(0);
    m_rightShooterMotor.set(0);
    m_rightFeedMotor.set(0);
  }

  public Command stopCommand() {
    return this.runOnce(
        () -> {
          stop();
        });
  }


  @Override
  public void periodic() {
    redLimitSwitch = (m_leftFeedMotor.isFwdLimitSwitchClosed() == 1);
    blackLimitSwitch = (m_leftFeedMotor.isRevLimitSwitchClosed() == 1);
    SmartDashboard.putBoolean("Red Switch", redLimitSwitch);
    SmartDashboard.putBoolean("Black Switch", blackLimitSwitch);
  }
   /*  // Get PID coefficients from SmartDashboard
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
*/
//ONLY LIMIT SWITCH THING
    //isLimitSwitchClosed = m_feedMotor.isFwdLimitSwitchClosed();

    //SmartDashboard.putNumber("ShooterVelocity", m_encoder.getVelocity());
    //SmartDashboard.putNumber("ShooterVoltage", m_shooterMotor.getBusVoltage());
    //SmartDashboard.putNumber("FeederSpeed", m_feedMotor.get());
}
  
  

