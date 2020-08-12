/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class ToggleIntake extends Command {
  private Intake intake;
  public ToggleIntake() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    intake = Robot.container.intake;
    requires(intake);
  }
  private Boolean position = false;
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("Toggling Intake");
    if(position == false) {
      intake.setMotor(20);
      intake.setPiston(true,false);
      position = true;
    }
    else {
      intake.setMotor(20);
      intake.setPiston(false,true);
      position = false;
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
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
