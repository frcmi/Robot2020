/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

public class ManualDrive extends Command {
  private DriveTrain driveTrain;
  private Joystick left;
  private Joystick right;
  private XboxController controller;

  public ManualDrive() {
    driveTrain = Robot.container.driveTrain;
    requires(driveTrain);
    /**
    left = Robot.container.leftJoystick;
    right = Robot.container.rightJoystick;
    */
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //driveTrain.tankDrive(left.getY(), right.getY());
    //driveTrain.arcadeDrive(right.getY(), right.getX());
    driveTrain.arcadeDrive(controller.getRawAxis(1), -controller.getRawAxis(5));
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
