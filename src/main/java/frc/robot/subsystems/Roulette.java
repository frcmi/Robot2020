/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Roulette extends Subsystem {
  public static class Constants{

  }

  private int desiredPosition; // Desired position, number from 0 to 4

  private TalonFX motor;
  private Encoder absoluteEncoder;

  // TODO @tong implement this
  public Roulette(){

  }

  //TODO @tong do this
  public void moveToPosition(int position){
    position %= 5;
    desiredPosition = position;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
