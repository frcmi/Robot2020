/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.shuffleboard.*;

import frc.robot.commands.Teleop;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  static class DriveConstants{
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = false;
    public static final boolean kLeftMotorReversed = true;
    public static final boolean kRightMotorReversed = false;
    public static final int[] kLeftEncoderPorts = {0, 1};
    public static final int[] kRightEncoderPorts = {2, 3};
    private static final int[] kLeftDeviceIds = new int[]{1, 2, 3};
    private static final int[] kRightDeviceIds = new int[]{4, 5, 6};
    public static final boolean kGyroReversed = true;
    private static final int kEncoderCPR = 1000; //TODO get this
    private static final double kWheelDiameterMeters = 0.1524;
    public static final double kEncoderDistancePerPulse = (kWheelDiameterMeters * Math.PI) / kEncoderCPR;
  }

  private SpeedController[] leftMotors, rightMotors;

  private SpeedControllerGroup left, right;

  private AHRS navx;

  private final DifferentialDrive diffDrive;

  // The odometry (position-tracker)
  private DifferentialDriveOdometry odometry;

  private Encoder leftEncoder, rightEncoder;

  public DriveTrain(){
    super();
    leftMotors = new SpeedController[DriveConstants.kLeftDeviceIds.length];
    for(int i=0; i<DriveConstants.kLeftDeviceIds.length; i++){
      leftMotors[i] = new WPI_TalonFX(DriveConstants.kLeftDeviceIds[i]);
    }

    rightMotors = new SpeedController[DriveConstants.kRightDeviceIds.length];
    for(int i=0; i<DriveConstants.kRightDeviceIds.length; i++){
      rightMotors[i] = new WPI_TalonFX(DriveConstants.kRightDeviceIds[i]);
    }

    //Sets speed controller groups and differential drive
    left = new SpeedControllerGroup(leftMotors[0], leftMotors[1], leftMotors[2]);
    right = new SpeedControllerGroup(rightMotors[0], rightMotors[1], rightMotors[2]);

    left.setInverted(DriveConstants.kLeftMotorReversed);
    right.setInverted(DriveConstants.kRightMotorReversed);
    diffDrive = new DifferentialDrive(left, right);

    //Sets NavX
    navx = new AHRS(SPI.Port.kMXP);
    resetHeading();

    //Sets odometry
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()), new Pose2d(0.0, 0.0, new Rotation2d()));

    //Sets encoders
    leftEncoder = new Encoder(DriveConstants.kLeftEncoderPorts[0], DriveConstants.kLeftEncoderPorts[1], DriveConstants.kLeftEncoderReversed);
    rightEncoder = new Encoder(DriveConstants.kRightEncoderPorts[0], DriveConstants.kRightEncoderPorts[1], DriveConstants.kRightEncoderReversed);
    leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    resetEncoders();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Get my gyro angle. We are negating the value because gyros return positive
    // values as the robot turns clockwise. This is not standard convention that is
    // used by the WPILib classes.
    Rotation2d gyroAngle = Rotation2d.fromDegrees(-navx.getAngle());

    // Update the pose
    double leftDistance = leftEncoder.getDistance();
    double rightDistance = rightEncoder.getDistance();

    odometry.update(gyroAngle, leftDistance, rightDistance);
  }

  /**
   * Basic Tank Drive method for drive subsystem, takes direct inputs for left and
   * right sides.
   * 
   * @param leftPower    value from -1 to 1 set to left motor group. + is forward.
   * @param rightPower   value from -1 to 1 set to right motor group. + is
   *                     forward.
   * @param squareInputs squares motor inputs if true
  */
  public void tankDrive(double leftPower, double rightPower, boolean squareInputs) {
    diffDrive.tankDrive(leftPower, rightPower, squareInputs);
  }

  /**
   * Tank Drive with volatge inputs instead of power inputs.
   * 
   * Voltage inputs allows more control over motors.
   * 
   * @param leftVolts  Value from -12 to 12 volts set to left motor group. + is
   *                   forward.
   * @param rightVolts Value from -12 to 12 volts set to right motor group. + is
   *                   forward.
  */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    left.setVoltage(leftVolts);
    right.setVoltage(-rightVolts);
    diffDrive.feed();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
  */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, pose.getRotation());
  }

  /**
   * Resets Encoders
  */
  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  /**
   * Zeroes the heading of the robot.
  */
  public void resetHeading() {
    navx.reset();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new Teleop());
  }

  public void addShuffleBoardTab() {
    ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");

    // Add test buttons to a layout in the tab
    ShuffleboardLayout commandsLayout = driveTab.getLayout("Test", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 3);

    //commandsLayout.add("Turn to 90", new AutoTurnToHeading(this).withMaxPower(0.35).toHeading(90));

    // Add the DifferentialDrive object and encoders to a list layout in the tab.
    ShuffleboardLayout diffDriveLayout = driveTab.getLayout("Base", BuiltInLayouts.kList).
      withPosition(2, 0).
      withSize(4, 5);

    diffDriveLayout.add("Differential Drive", diffDrive).withWidget(BuiltInWidgets.kDifferentialDrive);
    diffDriveLayout.add("Left Encoder", leftEncoder).withWidget(BuiltInWidgets.kEncoder);
    diffDriveLayout.add("Right Encoder", rightEncoder).withWidget(BuiltInWidgets.kEncoder);

    // Add the odometry to a layout in the tab.
    ShuffleboardLayout positionLayout = driveTab.getLayout("Position", BuiltInLayouts.kList).
      withPosition(6, 0).
      withSize(2, 2);

    positionLayout.addNumber("X", () -> getPose().getTranslation().getX());
    positionLayout.addNumber("Y", () -> getPose().getTranslation().getY());
    positionLayout.addNumber("Heading", () -> getHeadingContinuous());
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
  */
  public double getHeading() {
    return Math.IEEEremainder(this.navx.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * returns the continuous heading of the robot
   * 
   * @return the continuous heading of the robot
  */
  public double getHeadingContinuous() {
    return navx.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
  */
  public double getTurnRate() {
    return navx.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * 
   * @return The x,y position of the robot
  */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

}