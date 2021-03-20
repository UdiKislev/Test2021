// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FeedForward;

public class Chassis extends SubsystemBase {

  public static final double PULSE_PER_METER = 44700;

  /** Creates a new ExampleSubsystem. */
  public Chassis() {
    m_odometry = new DifferentialDriveOdometry(rotaionHeading());
    left2.follow(left1);
    right2.follow(right1);
  }

  private WPI_TalonFX left1 = new WPI_TalonFX(Constants.Left1Motor);
  private WPI_TalonFX left2 = new WPI_TalonFX(Constants.Left2Motor);
  private WPI_TalonFX right1 = new WPI_TalonFX(Constants.Right1Motor);
  private WPI_TalonFX right2 = new WPI_TalonFX(Constants.Right2Motor);
  
  // The motors on the right side of the drive.
  private final SpeedControllerGroup m_leftMotors =
      new SpeedControllerGroup(left1,left2);
  private final SpeedControllerGroup m_rightMotors =
      new SpeedControllerGroup(right1, right2);

  public double leftPosition() {
    return left1.getSelectedSensorPosition();
  }
  public double rightPosition() {
    return right1.getSelectedSensorPosition();
  }

  public static double encoderPositionToDistance(double encoderPosition) {
    return encoderPosition/PULSE_PER_METER;
  }
  public static double talonVelocityToVelocity(double talonVelocity) {
    return 10*talonVelocity/PULSE_PER_METER;
  }
  public static double velocityToTalonVelocity(double velocity) {
    return velocity * PULSE_PER_METER / 10;
  }

  public double leftDistance() {
    return encoderPositionToDistance(leftPosition());
  }
  public double rightDistance() {
    return encoderPositionToDistance(rightPosition());
  }
  public double leftVelocity() {
    return talonVelocityToVelocity(left1.getSelectedSensorVelocity());
  }
  public double rightVelocity() {
    return talonVelocityToVelocity(right1.getSelectedSensorVelocity());
  }
  
  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // The left-side drive encoder
  // The gyro sensor
  private final TalonSRX gyroTalon = new TalonSRX(10);
  private final PigeonIMU m_gyro = new PigeonIMU(gyroTalon);
  public double heading() {
    return m_gyro.getFusedHeading();    
  }

  public Rotation2d rotaionHeading() {
    return new Rotation2d(Math.toRadians(heading()));
  }

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;


  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(rotaionHeading(), leftDistance(), rightDistance());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftVelocity(),rightVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose,rotaionHeading());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(-rightVolts);
    m_drive.feed();
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    left1.setSelectedSensorPosition(0);
    right1.setSelectedSensorPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getDistance() {
    return (leftDistance() + rightDistance()) / 2.0;
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    m_gyro.setFusedHeading(0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    double[] xyz = new double[3];
    m_gyro.getRawGyro(xyz);
    return -xyz[2];
  }

  public void setVelocity(double leftVelocity, double rightVelocity) {
    double lff = FeedForward.feedForwardLeftPower(leftVelocity, rightVelocity);
    double rff = FeedForward.feedForwardRightPower(leftVelocity, rightVelocity);
    left1.set(ControlMode.Velocity, velocityToTalonVelocity(leftVelocity), DemandType.ArbitraryFeedForward, lff);
    right1.set(ControlMode.Velocity, velocityToTalonVelocity(rightVelocity), DemandType.ArbitraryFeedForward, rff);
  }
}
  
