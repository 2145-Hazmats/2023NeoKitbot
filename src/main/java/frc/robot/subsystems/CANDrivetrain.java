// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
 import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
/* This class declares the subsystem for the robot drivetrain if controllers are connected via CAN. Make sure to go to
 * RobotContainer and uncomment the line declaring this subsystem and comment the line for PWMDrivetrain.
 *
 * The subsystem contains the objects for the hardware contained in the mechanism and handles low level logic
 * for control. Subsystems are a mechanism that, when used in conjuction with command "Requirements", ensure
 * that hardware is only being used by 1 command at a time.
 */
public class CANDrivetrain extends SubsystemBase {
  /*Class member variables. These variables represent things the class needs to keep track of and use between
  different method calls. */
  DifferentialDrive m_drivetrain;
  public static final ADIS16470_IMU gyro = new ADIS16470_IMU();
  /*Constructor. This method is called when an instance of the class is created. This should generally be used to set up
   * member variables and perform any configuration or set up necessary on hardware.
   */
  //WPI_TalonSRX leftFront = new WPI_TalonSRX(kLeftFrontID);
  //WPI_TalonSRX rightFront = new WPI_TalonSRX(kRightFrontID);

  CANSparkMax leftFront = new CANSparkMax(kLeftFrontID, MotorType.kBrushless);
  CANSparkMax rightFront = new CANSparkMax(kRightFrontID, MotorType.kBrushless);
  private RelativeEncoder m_leftMotorEncoder = leftFront.getEncoder();
  private RelativeEncoder m_rightMotorEncoder = rightFront.getEncoder();

  double targetAngle = 0;

  private final double MotorTick2Feets = (6*Math.PI)/(12*12.75);

  private double gyroPosition;
  private double leftPosition; 
  private double rightPosition;
  public double distance;
  
  public CANDrivetrain() {
    m_rightMotorEncoder.setPositionConversionFactor(MotorTick2Feets);
    m_leftMotorEncoder.setPositionConversionFactor(MotorTick2Feets);
    m_rightMotorEncoder.setPosition(0);
    m_leftMotorEncoder.setPosition(0);

    //leftFront.enableVoltageCompensation(11.5);
    //rightFront.enableVoltageCompensation(11.5);
   
    //m_drivetrain.setDeadband(0.025);
    // m_drivetrain.setSafetyEnabled(false);
    // CANSparkMax leftRear = new CANSparkMax(kLeftRearID, MotorType.kBrushed);

    // CANSparkMax rightRear = new CANSparkMax(kRightRearID, MotorType.kBrushed);

    /*Sets current limits for the drivetrain motors. This helps reduce the likelihood of wheel spin, reduces motor heating
     *at stall (Drivetrain pushing against something) and helps maintain battery voltage under heavy demand */
    // leftFront.setSmartCurrentLimit(kCurrentLimit);
    // leftRear.setSmartCurrentLimit(kCurrentLimit);
    // rightFront.setSmartCurrentLimit(kCurrentLimit);
    // rightRear.setSmartCurrentLimit(kCurrentLimit);

    // Set the rear motors to follow the front motors.
    // leftRear.follow(leftFront);
    // rightRear.follow(rightFront);

    // Invert the left side so both side drive forward with positive motor outputs
    leftFront.setInverted(true);
    rightFront.setInverted(false);

    // Put the front motors into the differential drive object. This will control all 4 motors with
    // the rears set to follow the fronts
    m_drivetrain = new DifferentialDrive(leftFront, rightFront);
  }

  public void driveForward(double speed) {
    leftFront.set(speed);
    rightFront.set(speed);
  }

  public void driveTurn(double speed) {
    leftFront.set(-speed);
    rightFront.set(speed);
  }

  public void FaceTowardSubwoofer(double Xcurrentpose, double Ycurrentpose) {
    double side1 = Constants.DrivetrainConstants.xsubwoofer - Xcurrentpose;
    double side2 = Constants.DrivetrainConstants.ysubwoofer - Ycurrentpose;
    targetAngle = Math.atan(side2/side1);
  }


  public void FaceStraight(double currentrotation) {
    double error =  -targetAngle - currentrotation;

    
    leftFront.set(-error*.0525); 
    rightFront.set(error*.0525); 

  }

  /*Method to control the drivetrain using arcade drive. Arcade drive takes a speed in the X (forward/back) direction
   * and a rotation about the Z (turning the robot about it's center) and uses these to control the drivetrain motors */
  public void arcadeDrive(double speed, double rotation) {
    m_drivetrain.arcadeDrive(speed, rotation);
  }

  public void setDrivetrainSafety(boolean isEnabled) {
    m_drivetrain.setSafetyEnabled(isEnabled);
  }

  public void FeedDrivetrain() {
    m_drivetrain.feed();
  }

  public void ResetEncoders() {
    m_leftMotorEncoder.setPosition(0);
    m_rightMotorEncoder.setPosition(0);
  }

  public double getGyro() {
    return gyroPosition* Constants.DrivetrainConstants.GyrotoDeg;
  }

  @Override
  public void periodic() {
    leftPosition = m_leftMotorEncoder.getPosition()*MotorTick2Feets;
    rightPosition = m_rightMotorEncoder.getPosition()*MotorTick2Feets;
    distance = (m_leftMotorEncoder.getPosition() + m_rightMotorEncoder.getPosition()) / 2;
    gyroPosition = gyro.getAngle(IMUAxis.kY);

    SmartDashboard.putNumber("Gyro", gyroPosition);
    SmartDashboard.putNumber("Left Drive Motor Encoder Value in Feets", m_leftMotorEncoder.getPosition());
    SmartDashboard.putNumber("Right Drive Motor Encoder Value in Feets", m_rightMotorEncoder.getPosition());
    SmartDashboard.putNumber("Average Drive Motor Encoder Value in Feets", distance);
    SmartDashboard.putNumber("NEW Targetangle", targetAngle );
    /*This method will be called once per scheduler run. It can be used for running tasks we know we want to update each
     * loop such as processing sensor data. Our drivetrain is simple so we don't have anything to put here */
  }

  public Command driveForwardCommand(double speed){
    return new StartEndCommand(() -> this.driveForward(speed), () -> this.driveForward(0.0),
    this);
  }
}
