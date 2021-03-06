/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.Robot;
import frc.robot.subsystems.Roulette;

//Moves the roulette to the currently selected hole
public class MaintainRoulettePosition extends Command {
  public static class Constants{
    public static final int errorThreshold = 10;
    public static final long timeInThresholdToFinishNs = 200000000;

    public static final double kP = 1;
    public static final double kI = 0.00001;
    public static final double kD = 5;
  }
  private Roulette roulette;
  private PIDController controller;

  public MaintainRoulettePosition() {
    roulette = Robot.container.roulette;
    requires(roulette);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    controller = new PIDController(Constants.kP, Constants.kI, Constants.kD);
    controller.enableContinuousInput(0, Roulette.Constants.kTicksPerRevolution-1);
    controller.setSetpoint(roulette.getEncoderSetpoint());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(controller.getSetpoint() != roulette.getEncoderSetpoint()){
      controller.setSetpoint(roulette.getEncoderSetpoint());
    }
    roulette.setMotor(controller.calculate(roulette.getEncoderValue()));
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
