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
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class Shooter extends Subsystem {
  private static class Constants {
    public static final int topID = 7;
    public static final int bottomID = 8;
    public static final int actuatorID = 9;
    public static final double speedThreshold = 100;
    public static final double angleThreshold = 5*Math.PI/180;
    public static final double flywheelNeutralDeadband = 0.001;
    public static final int timeoutMs = 30;
    public static final double ticksPerRevolution = 2048.0;
    public static final double flywheelIz = 300;
    public static final double flywheelPeakOut = 1.00;
    public static final double rpmToUnitsPer100ms = ticksPerRevolution/600.0;

    public static final double shooterAngleDifference = -0.0680678;
    public static final double shooterConstantA = 0.4366;
    public static final double shooterConstantB = 0.4477;
    public static final double pistonYOffset = 0.01588;
    public static final double errorBound = 0.01;
    public static final double actuatorMinLength = 0.3334+errorBound;
    public static final double actuatorMaxLength = 0.8477-errorBound;
    public static final double shooterConstant1 = Math.pow(shooterConstantA, 2) + Math.pow(shooterConstantB, 2) - Math.pow(pistonYOffset, 2); // A^2+B^2-y^2
    public static final double shooterConstant2 = 2*shooterConstantA*shooterConstantB; // 2AB
    public static final double actuatorMetersPerRadian = 0.00551254848;
    public static final double actuatorMetersPerTick = actuatorMetersPerRadian*2*Math.PI/ticksPerRevolution;

    public static final double flywheelkP = 0.1;
    public static final double flywheelkI = 0.000001;
    public static final double flywheelkD = 5;
    public static final double flywheelkF = 1023.0/20660.0; //1023 represents output value to Talon at 100%, 20660 represents Velocity units at 100% output

    public static final double actuatorkP = 0.1;
    public static final double actuatorkI = 0.000001;
    public static final double actuatorkD = 5;
    public static final double actuatorkF = 10;
    public static final boolean actuatorSensorPhase = true;
    public static final boolean actuatorMotorInvert = false;
  }
  private TalonFX top, bottom, actuator;
  private double desiredSpeed, desiredAngle;
  private Compressor compressor;
  private DoubleSolenoid piston;

  public Shooter(){
    desiredSpeed = 0;
    desiredAngle = 0;
    top = new TalonFX(Constants.topID);
    bottom = new TalonFX(Constants.bottomID);
    actuator = new TalonFX(Constants.actuatorID);
    compressor = new Compressor(0);
<<<<<<< HEAD
    piston = new DoubleSolenoid(3, 4);
=======
    //piston = new DoubleSolenoid(solenoidValues);
>>>>>>> 3c3b6553be0b3f9b2f2320948ee5d6ecbb7f673f

    /* ---- Flywheel Config ---- */
    bottom.setInverted(true);

    top.configFactoryDefault();
    bottom.configFactoryDefault();

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

    //Configures the Compressor for pneumatics
    compressor.setClosedLoopControl(true);
    compressor.start();


    /* ----Actuator config---- */
    actuator.configFactoryDefault();

    actuator.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.timeoutMs);

    actuator.setSensorPhase(Constants.actuatorSensorPhase);

    actuator.setInverted(Constants.actuatorMotorInvert);

    actuator.configNominalOutputForward(0, Constants.timeoutMs);
    actuator.configNominalOutputReverse(0, Constants.timeoutMs);
    actuator.configClosedLoopPeakOutput(1, Constants.timeoutMs);

    actuator.configAllowableClosedloopError(0, 0, Constants.timeoutMs);

    actuator.config_kF(0, Constants.actuatorkF, Constants.timeoutMs);
    actuator.config_kP(0, Constants.actuatorkP, Constants.timeoutMs);
    actuator.config_kI(0, Constants.actuatorkI, Constants.timeoutMs);
    actuator.config_kD(0, Constants.actuatorkD, Constants.timeoutMs);
  }

  //Sets flywheel to speed given in rpm
  public void setFlywheelSpeed(double speed){
    desiredSpeed = speed;
    top.set(TalonFXControlMode.Velocity, speed*Constants.rpmToUnitsPer100ms);
    bottom.set(TalonFXControlMode.Velocity, speed*Constants.rpmToUnitsPer100ms);
  }

  //Stops all motors
  public void stop(){
    desiredSpeed = 0;
    top.set(TalonFXControlMode.PercentOutput, 0);
    bottom.set(TalonFXControlMode.PercentOutput, 0);
    actuator.set(TalonFXControlMode.PercentOutput, 0);
  }

  //converts angle to actuator length
  public double angleToActuatorLength(double angle){
    return Math.sqrt(Constants.shooterConstant1 - Constants.shooterConstant2*Math.cos(angle-Constants.shooterAngleDifference));
  }

  //converts actuator length to angle
  public double actuatorLengthToAngle(double actuatorOffset){
    return Math.acos((Constants.shooterConstant1-Math.pow(actuatorOffset, 2))/Constants.shooterConstant2)+Constants.shooterAngleDifference;
  }

  //Sets actuator length setpoint to given value in meters
  public void setActuatorLength(double length){
    if(length > Constants.actuatorMaxLength) length = Constants.actuatorMaxLength;
    if(length < Constants.actuatorMinLength) length = Constants.actuatorMinLength;
    double lengthDifference = length - Constants.actuatorMinLength;
    double ticks = lengthDifference/Constants.actuatorMetersPerTick;
    actuator.set(TalonFXControlMode.Position, ticks);
  }

  //Sets the shooter setpoint to an angle given in radians
  public void setAngle(double angle){
    desiredAngle = angle;
    setActuatorLength(angleToActuatorLength(angle));
  }

  //Returns current angle of the shooter in radians
  public double getAngle(){
    double pos = actuator.getSelectedSensorPosition();
    double angle = actuatorLengthToAngle(Constants.actuatorMinLength+Constants.actuatorMetersPerTick*pos);
    return angle;
  }

  //Returns current speed of flywheel in rpm
  public double getTopFlywheelSpeed(){
    double topRawSpeed = top.getSelectedSensorVelocity();
    double topSpeed = topRawSpeed/Constants.rpmToUnitsPer100ms;
    return topSpeed;
  }

  public double getBottomFlywheelSpeed(){
    double bottomRawSpeed = bottom.getSelectedSensorVelocity();
    double bottomSpeed = bottomRawSpeed/Constants.rpmToUnitsPer100ms;
    return bottomSpeed;
  }

  // Returns whether the speed and angle are within requirements to shoot
  public boolean isReady(){
    return Math.abs(getTopFlywheelSpeed()-desiredSpeed) <= Constants.speedThreshold && 
            Math.abs(getBottomFlywheelSpeed()-desiredSpeed) <= Constants.speedThreshold && 
            Math.abs(getAngle()-desiredAngle) <= Constants.angleThreshold;
  }

  //turns piston off
  public void turnPistonOff() {
    piston.set(kOff);
  }
  
  // gets piston value
  public DoubleSolenoid.Value getPistonValue() {
    return piston.get();
  }

  //moves the piston to boop the ball
  public void movePiston() {
    piston.set(kForward);

  }

  //moves the piston in reverse
  public void movePistonInReverse() {
    piston.set(kReverse);
  }


  @Override
  public void initDefaultCommand() {

  }
}