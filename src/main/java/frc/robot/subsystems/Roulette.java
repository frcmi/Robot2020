/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;


/**
 * Add your docs here.
 */
public class Roulette extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static int[] ballLocation = new int[5];
  //array maps to the roulette wheel in a clockwise fashion
  //ballLocation[0] is the top (slot that ejects the ball)
  //ballLocation[3] is the bottom (slot where balls are taken in)

  
  private static boolean isEmpty = true;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());

  }

  public void cycle(int[] arr) {
    //makes sure that ballLocation[0] is full
    //makes sure that ballLocation[2] is empty
    //optimizes shooting

  }

  public void update() {
    //checks color sensors and updates ballLocation accordingly
    //keeps track of ball position
    //might scrap because no sensors
    /**requires(ultrasonicSensor);
    if (ultrasonicSensor1.read() < 5) {
      ballLocation[0] = 1;
    } else {
      ballLocation[0] = 0;
    }
    if (ultrasonicSensor2.read() < 5) {
      ballLocation[1] = 1;
    } else {
      ballLocation[1] = 0;
    }
    if (ultrasonicSensor3.read() < 5) {
      ballLocation[2] = 1;
    } else {
      ballLocation[2] = 0;
    }
    if (ultrasonicSensor4.read() < 5) {
      ballLocation[3] = 1;
    } else {
      ballLocation[3] = 0;
    }
    if (ultrasonicSensor5.read() < 5) {
      ballLocation[4] = 1;
    } else {
      ballLocation[4] = 0;
    }
    cycle();
    **/
  }

  public void addBall() {
    //adds a ball to ballLocation[3]
    ballLocation[3] = 1;
    isEmpty = false;
    update(); //checks whether or not it's a successful load
  
  }

  public void eject() {
    //removes a ball from ballLocation[0]
    ballLocation[0] = 0;
    if (ballLocation[1] == 0 && ballLocation[2] == 0 && ballLocation[3] == 0 && ballLocation[4] == 0) {
      //probably better way to check, if so please edit this
      isEmpty = true;
    }
    update();
  
  }

}
