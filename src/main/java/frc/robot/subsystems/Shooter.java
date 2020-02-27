/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Shooter extends Subsystem {
  private static class Constants {
    public static final int topID = 7;
    public static final int bottomID = 8;
    public static final int actuatorID = 9;
    public static final double speedThreshold = 0;
    public static final double angleThreshold = 0;
  }

  private TalonFX top, bottom, actuator;
  private double desiredSpeed, desiredAngle;

  public Shooter(){
    top = new TalonFX(Constants.topID);
    bottom = new TalonFX(Constants.bottomID);
    actuator = new TalonFX(Constants.actuatorID);
    top.setInverted(true);
    desiredSpeed = 0;
    desiredAngle = 0;
  }

  //Sets flywheel to speed given in position moved per 100ms
  public void setFlywheelSpeed(double speed){
    desiredSpeed = speed;
    top.set(TalonFXControlMode.Velocity, speed);
    bottom.set(TalonFXControlMode.Velocity, speed);
  }

  public void stop(){
    desiredSpeed = 0;
    top.set(TalonFXControlMode.PercentOutput, 0);
    bottom.set(TalonFXControlMode.PercentOutput, 0);
  }

  //TODO @tong sets the shooter to an angle given in radians
  public void setAngle(double angle){
    desiredAngle = angle;
  }

  //TODO @tong gets current angle of the shooter
  public double getAngle(){
    return 0;
  }

  //TODO @tong returns current speed of flywheel
  public double getSpeed(){
    return 0;
  }

  // Returns whether the speed and angle are within requirements to shoot
  public boolean isReady(){
    return Math.abs(getSpeed()-desiredSpeed) <= Constants.speedThreshold && Math.abs(getAngle()-desiredAngle) <= Constants.angleThreshold;
  }

  @Override
  public void initDefaultCommand() {

  }
}