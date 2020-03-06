/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Camera;

public class ToggleCamera extends Command {
  private Camera camera;
  private boolean debug;

  public ToggleCamera() {
    camera = Robot.container.camera;
    debug = false;
  }

  public ToggleCamera(boolean debug){
    this();
    this.debug = debug;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if(debug){
      camera.debugCamera();
    } else{
      camera.toggleCamera();
    }
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
