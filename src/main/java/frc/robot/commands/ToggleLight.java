/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Camera;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ToggleLight extends Command {
  private Camera camera;
  public ToggleLight() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    camera = Robot.container.camera;
    requires(camera);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("Toggling camera");
    camera.toggle();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
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