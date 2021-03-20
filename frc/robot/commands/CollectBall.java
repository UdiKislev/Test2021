// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Chassis;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class CollectBall extends CommandBase {

  
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  public final Chassis chassis;
  Translation2d centerToCamera = new Translation2d(Constants.BallCameraToCenterDiatance, Rotation2d.fromDegrees(Constants.BallCameraToCenterAngle));

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public CollectBall(Chassis subsystem) {
    chassis = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double cameraAngle = SmartDashboard.getNumber(Constants.BallAngleString, 0.0);
    double cameraDistance = SmartDashboard.getNumber(Constants.BallDistanceString, 0.0);
    Translation2d cameraToBall = new Translation2d(cameraDistance, Rotation2d.fromDegrees(cameraAngle));
    Translation2d centerToCamera = new Translation2d(Constants.BallCameraToCenterDiatance, Rotation2d.fromDegrees(Constants.BallCameraToCenterAngle));
    Translation2d vec = centerToCamera.plus(cameraToBall);
    double distance = vec.getNorm();
    double sin = (new Rotation2d(vec.getX(), vec.getY())).getSin();
    double radius = distance/2/sin;
    double vel = 0.7;
    double ratio = radius / (radius + Constants.WHEEL_BASE/2);
    double leftVelocity = vel * (1 + ratio);
    double rightVelocity = vel * (1 - ratio);
    chassis.setVelocity(leftVelocity, rightVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
