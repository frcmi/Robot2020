/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.Teleop;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private static final int[] LEFT_DEVICE_IDS = new int[]{1, 2, 3};
  private static final int[] RIGHT_DEVICE_IDS = new int[]{4, 5, 6};

  private TalonFX[] left;
  private TalonFX[] right;

  public DriveTrain(){
    super();
    left = new TalonFX[LEFT_DEVICE_IDS.length];
    for(int i=0; i<LEFT_DEVICE_IDS.length; i++){
      left[i] = new TalonFX(LEFT_DEVICE_IDS[i]);
    }

    right = new TalonFX[RIGHT_DEVICE_IDS.length];
    for(int i=0; i<RIGHT_DEVICE_IDS.length; i++){
      right[i] = new TalonFX(RIGHT_DEVICE_IDS[i]);
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new Teleop());
  }

  public void setLeft(double value){
    for(int i=0; i<left.length; i++){
      left[i].set(ControlMode.PercentOutput, value);
    }
  }

  public void setRight(double value){
    for(int i=0; i<right.length; i++){
      right[i].set(ControlMode.PercentOutput, -value);
    }
  }
}