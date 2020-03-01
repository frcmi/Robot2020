/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
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
    public static final double flywheelNeutralDeadband = 0.001;
    public static final int timeoutMs = 30;
    public static final double flywheelkP = 0.1;
    public static final double flywheelkI = 0.001;
    public static final double flywheelkD = 5;
    public static final double flywheelkF = 1023.0/20660.0; //1023 represents output value to Talon at 100%, 20660 represents Velocity units at 100% output
    public static final double flywheelIz = 300;
    public static final double flywheelPeakOut = 1.00;
    public static final double rpmToUnitsPer100ms = 2048.0/600.0;
  }

  private TalonFX top, bottom, actuator;
  private double desiredSpeed, desiredAngle;

  public Shooter(){
    desiredSpeed = 0;
    desiredAngle = 0;
    top = new TalonFX(Constants.topID);
    bottom = new TalonFX(Constants.bottomID);
    actuator = new TalonFX(Constants.actuatorID);
    bottom.setInverted(true);

    top.configFactoryDefault();
    bottom.configFactoryDefault();
    actuator.configFactoryDefault();

    top.configNeutralDeadband(Constants.flywheelNeutralDeadband);
    bottom.configNeutralDeadband(Constants.flywheelNeutralDeadband);

    top.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.timeoutMs);
    bottom.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.timeoutMs);

    //Configures peak outputs
    top.configNominalOutputForward(0, Constants.timeoutMs);
    top.configNominalOutputReverse(0, Constants.timeoutMs);
    top.configClosedLoopPeakOutput(1, Constants.timeoutMs);

    bottom.configNominalOutputForward(0, Constants.timeoutMs);
    bottom.configNominalOutputReverse(0, Constants.timeoutMs);
    bottom.configClosedLoopPeakOutput(1, Constants.timeoutMs);

    //Configures PID constants
    top.config_kF(0, Constants.flywheelkF, Constants.timeoutMs);
    top.config_kP(0, Constants.flywheelkP, Constants.timeoutMs);
    top.config_kI(0, Constants.flywheelkI, Constants.timeoutMs);
    top.config_kD(0, Constants.flywheelkD, Constants.timeoutMs);

    bottom.config_kF(0, Constants.flywheelkF, Constants.timeoutMs);
    bottom.config_kP(0, Constants.flywheelkP, Constants.timeoutMs);
    bottom.config_kI(0, Constants.flywheelkI, Constants.timeoutMs);
    bottom.config_kD(0, Constants.flywheelkD, Constants.timeoutMs);



  }

  //Sets flywheel to speed given in rpm
  public void setFlywheelSpeed(double speed){
    desiredSpeed = speed;
    top.set(TalonFXControlMode.Velocity, speed*Constants.rpmToUnitsPer100ms);
    bottom.set(TalonFXControlMode.Velocity, speed*Constants.rpmToUnitsPer100ms);
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