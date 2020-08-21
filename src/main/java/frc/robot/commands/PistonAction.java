/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.Robot;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class PistonAction extends Command {

  private Shooter shooter;
  public PistonAction() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    shooter = Robot.container.shooter;
    requires(shooter);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    shooter.movePiston();
  }


  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
<<<<<<< HEAD:src/main/java/frc/robot/commands/PistonAction.java

    if (shooter.getPistonValue() == kForward) {
      return true;
    }
    return false;

=======
    /*if (shooter.getPistonValue()= kForward) {
      return true;
    }
    return false;
    */
    return true;
>>>>>>> 3c3b6553be0b3f9b2f2320948ee5d6ecbb7f673f:src/main/java/frc/robot/commands/MovePiston.java
  }


  // Called once after isFinished returns true
  @Override
  protected void end() {
    shooter.movePistonInReverse();
    //shooter.turnPistonOff();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
