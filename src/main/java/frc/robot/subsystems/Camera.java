/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * Add your docs here.
 */
public class Camera extends Subsystem {
  public static class CameraConstants{
    public static final int pcmID = 11;
    public static final int pcmPort = 0;
  }

  Solenoid light;

  public Camera(){
    super();
    light = new Solenoid(CameraConstants.pcmID, CameraConstants.pcmPort);
    light.set(true);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void toggle() {
    light.set(!light.get());
  }
}
