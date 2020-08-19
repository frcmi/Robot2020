/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Intake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public static class Constants {



  }

  private TalonFX motor;
  private DoubleSolenoid piston;

  //needs port numbers
  public Intake() {
    motor = new TalonFX(00);
    //piston = new DoubleSolenoid(portnumbers);
  }
  
  //sets motor percent output
  public void setMotor(double value) {
    motor.set(TalonFXControlMode.PercentOutput, value);
  }

  /** 
  //sets piston position
  public void setPiston(Boolean firstVal, Boolean secondVal) {
    if(firstVal) {
      piston.set(Value.kForward);
    }
    else if(secondVal) {
      piston.set(Value.kReverse);
    }
  }
  */

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
