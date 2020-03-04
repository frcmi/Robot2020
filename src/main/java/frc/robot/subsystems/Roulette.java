/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.controller.PIDController;

/**
 * Add your docs here.
 */
public class Roulette extends Subsystem {
  public static class Constants{
    public static final int[] kEncoderPorts = {0, 1};
    public static final boolean kEncoderReversed = false;

    public static final int motorID = 10;

    public static final int kTicksPerRevolution = 1024;
    public static final int kTicksPerHole = kTicksPerRevolution/5+1;
    public static final int kAlignmentOffset = 0 % kTicksPerHole;
  }

  private int currentPos; // Desired position, number from 0 to 4

  private TalonFX motor;
  private Encoder encoder;

  // TODO @tong implement this
  public Roulette(){
    motor = new TalonFX(Constants.motorID);

    encoder = new Encoder(Constants.kEncoderPorts[0], Constants.kEncoderPorts[1], Constants.kEncoderReversed);

    int encoderPos = encoder.get();

    // Sets setpoint to nearest hole

    if(encoderPos % Constants.kTicksPerHole <= Constants.kTicksPerHole/2){
      currentPos = encoderPos/Constants.kTicksPerHole;
    } else{
      currentPos = (encoderPos/Constants.kTicksPerHole+1)%5;
    }


  }

  //TODO @tong do this
  public double getEncoderValue(){
    return encoder.get();
  }

  public void setMotor(double value){
    motor.set(TalonFXControlMode.PercentOutput, value);
  }

  //Gets the current setpoint position for the roulette
  public int getCurrentPos(){
    return currentPos;
  }

  //Gets the desired encoder value for currentPos
  public int getEncoderSetpoint(){
    return currentPos*Constants.kTicksPerHole;
  }

  //Sets the roulette setpoint
  public void setCurrentPos(int pos){
    currentPos = pos % 5;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}