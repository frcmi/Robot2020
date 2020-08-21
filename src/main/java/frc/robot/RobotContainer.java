package frc.robot;

import java.io.IOException;
import java.util.List;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Roulette;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Intake;
import frc.robot.commands.InterruptAll;
import frc.robot.commands.MaintainRoulettePosition;
import frc.robot.commands.ManualDrive;
import frc.robot.commands.SetFlywheelSpeed;
import frc.robot.commands.SetShooterAngle;
import frc.robot.commands.ToggleLight;
import frc.robot.commands.ToggleIntake;
import frc.robot.commands.PistonAction;
import frc.robot.commands.ChangeRoulettePosition;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // subsystems
  public final DriveTrain driveTrain = new DriveTrain();
  public final Camera camera = new Camera();
  public final Shooter shooter = new Shooter();
  public final Roulette roulette = new Roulette();
  public final Intake intake = new Intake();

  // Joystick and JoystickButtons
  /**
  public final Joystick leftJoystick = new Joystick(0);
  public final Joystick rightJoystick = new Joystick(1);

  public JoystickButton resetSensorsButton = new JoystickButton(rightJoystick, 3);
  public JoystickButton interruptAllButton = new JoystickButton(leftJoystick, 2);
  public JoystickButton toggleLightButton = new JoystickButton(leftJoystick, 3);
  public JoystickButton flywheelOnButton = new JoystickButton(leftJoystick, 1);
  public JoystickButton flywheelOffButton = new JoystickButton(rightJoystick, 1);
  public JoystickButton aimHighButton = new JoystickButton(rightJoystick, 4);
  public JoystickButton aimLowButton = new JoystickButton(leftJoystick, 4);
  */
  public final XboxController controller = new XboxController(1);

  public JoystickButton xButton = new JoystickButton(controller, 3);
	public JoystickButton yButton = new JoystickButton(controller, 4);
	public JoystickButton aButton = new JoystickButton(controller, 1);
	public JoystickButton bButton = new JoystickButton(controller, 2);
	public JoystickButton rightBumper = new JoystickButton(controller, 6);
  public JoystickButton leftBumper = new JoystickButton(controller, 5);
  public JoystickButton startButton = new JoystickButton(controller, 8);
	public JoystickButton selectButton = new JoystickButton(controller, 7);
	public JoystickButton leftStickButton = new JoystickButton(controller, 9);
	public JoystickButton rightStickButton = new JoystickButton(controller, 10);
  
  public JoystickButton interruptAllButton = xButton;
  public JoystickButton toggleLightButton = yButton;
  public JoystickButton flywheelOnButton = aButton;
  public JoystickButton flywheelOffButton = bButton;
  public JoystickButton aimHighButton = rightBumper;
  public JoystickButton aimLowButton = leftBumper;
  public JoystickButton toggleIntakeButton = startButton;

  // commands
  private ManualDrive manualDrive;
  private InterruptAll interruptAll;
  private ToggleLight toggleLight;
  private ToggleIntake toggleIntake;
  private MaintainRoulettePosition maintainRoulettePosition;
  private PistonAction pistonAction;

  // autonomous chooser
  private SendableChooser<AutoPath> autoPathChooser;

  private enum AutoPath {
    INITIATION_LINE_TO_MIDDLE("INITIATION_LINE_TO_MIDDLE.wpilib.json", new Pose2d(3.362, -3.989, new Rotation2d(0))),
    INITIATION_LINE_TO_LEFT_TRENCH("INITIATION_LINE_TO_LEFT_TRENCH.wpilib.json",
        new Pose2d(3.3, -0.786, new Rotation2d(0))),
    INITIATION_LINE_TO_RIGHT_TRENCH("INITIATION_LINE_TO_RIGHT_TRENCH.wpilib.json",
        new Pose2d(3.473, -7.501, new Rotation2d(0)));


    private final String fileName;
    private final Pose2d startingPosition;

    private AutoPath(String file, Pose2d position) {
      fileName = file;
      startingPosition = position;
    }

    public String getFile() {
      return fileName;
    }

    public Pose2d getStartingPosition() {
      return startingPosition;
    }
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
  }

  public void initialize(){
    System.out.println("RobotContainer");

    //Initialize commands
    manualDrive = new ManualDrive();
    interruptAll = new InterruptAll(driveTrain, camera, shooter);
    maintainRoulettePosition = new MaintainRoulettePosition();

    // subsystem default commands
    driveTrain.setDefaultCommand(manualDrive);
    roulette.setDefaultCommand(maintainRoulettePosition);

    // Configure the button bindings
    configureButtonBindings();
    driveTrain.addShuffleBoardTab();

    // Adds AutoPath chooser to SmartDashBoard
    ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
    autoPathChooser = new SendableChooser<AutoPath>();
    autoPathChooser.addOption(AutoPath.INITIATION_LINE_TO_MIDDLE.name(), AutoPath.INITIATION_LINE_TO_MIDDLE);
    autoPathChooser.addOption(AutoPath.INITIATION_LINE_TO_LEFT_TRENCH.name(), AutoPath.INITIATION_LINE_TO_LEFT_TRENCH);
    autoPathChooser.addOption(AutoPath.INITIATION_LINE_TO_RIGHT_TRENCH.name(), AutoPath.INITIATION_LINE_TO_RIGHT_TRENCH);
    autoTab.add("autoPath", autoPathChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    interruptAllButton.whenPressed(interruptAll);
    toggleLightButton.whenPressed(toggleLight);
    flywheelOnButton.whenPressed(new SetFlywheelSpeed(4000));
    flywheelOffButton.whenPressed(new SetFlywheelSpeed(0));
    aimHighButton.whenPressed(new SetShooterAngle(Math.PI/2));
    aimLowButton.whenPressed(new SetShooterAngle(0));
    toggleIntakeButton.whenPressed(toggleIntake);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    AutoPath path = autoPathChooser.getSelected();
    //Scheduler.getInstance().add(new SetStartPosition(drive, path.getStartingPosition()));
    
    return new ManualDrive();
    /*
    try {
      return new FollowPathWeaverFile(drive, path.getFile());
    } catch (IOException e) {
      e.printStackTrace();
      return null;
    }
    */
  }

  public void resetSensors() {
    driveTrain.resetHeading();
    driveTrain.resetOdometry(new Pose2d(0,0, new Rotation2d()));
  }
}