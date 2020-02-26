/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * Add your docs here.
 */
public class Camera extends Subsystem {
  private static class Constants {
    public static final int pcmID = 11;
    public static final int pcmPort = 7;
  }

  Solenoid light;
  NetworkTable opsi;

  public Camera() {
    super();
    opsi = NetworkTableInstance.getDefault().getTable("OpenSight");
    light = new Solenoid(Constants.pcmID, Constants.pcmPort);
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

  public Pose2d getPose(){
    Pose2d p = new Pose2d(opsi.getEntry("position-x").getDouble(0), 
                          opsi.getEntry("position-y").getDouble(0), 
                          Rotation2d.fromDegrees(opsi.getEntry("camera-angle").getDouble(0)));
    
    if(p.getTranslation().getX() == 0.0 && p.getTranslation().getY() == 0.0){
      return null;
    } else{
      return p;
    }
  }
}
