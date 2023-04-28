// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Auto_Complex;
import frc.robot.commands.Auto_DriveForward;
//import frc.robot.commands.Autos;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.TurnToAngleProfiled;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Power;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems are defined here...
  private final Drivetrain m_driveTrain = new Drivetrain();
  private final Power m_pdp = new Power();

  // Commands are defined here...
  // A simple auto routine that drives forward a specified distance, and then stops.
  private final Command m_simpleAuto =
      new Auto_DriveForward(
          AutoConstants.kAutoDriveDistanceMeters, AutoConstants.kAutoDriveSpeed, m_driveTrain);

  // A complex auto routine that drives forward, drops a hatch, and then drives backward.
  private final Command m_complexAuto = 
      new Auto_Complex(m_driveTrain);




  // Controllers
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);


  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    m_driveTrain.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new DriveWithJoysticks(
            m_driveTrain,
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getRightX()));


    // Add commands to the autonomous command chooser
    m_chooser.setDefaultOption("Simple Auto", m_simpleAuto);
    m_chooser.addOption("Complex Auto", m_complexAuto);

    SmartDashboard.putData(m_chooser);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
  controller_telemetry();




   // Drive at half speed when the right bumper is held
   m_driverController.rightBumper()
     .onTrue(new InstantCommand(() -> m_driveTrain.setMaxOutput(0.5)))
     .onFalse(new InstantCommand(() -> m_driveTrain.setMaxOutput(1)));

// Stabilize robot to drive straight with gyro when left bumper is held
  m_driverController.leftBumper()
   .whileTrue(
       new PIDCommand(
           new PIDController(
               DrivetrainConstants.kP_Stabilization,
               DrivetrainConstants.kI_Stabilization,
               DrivetrainConstants.kD_Stabilization),
           // Close the loop on the turn rate
           m_driveTrain::getTurnRate,
           // Setpoint is 0
           0,
           // Pipe the output to the turning controls
           output -> m_driveTrain.arcadeDrive(-m_driverController.getLeftY(), output),
           // Require the robot drive
           m_driveTrain));

  // Turn to 90 degrees when the 'X' button is pressed, with a 5 second timeout
  m_driverController.x()
   .onTrue(new TurnToAngle(90, m_driveTrain).withTimeout(5));

  // Turn to -90 degrees with a profile when the Circle button is pressed, with a 5 second timeout
  m_driverController.y()
   .onTrue(new TurnToAngleProfiled(-90, m_driveTrain).withTimeout(5));


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_chooser.getSelected();
  }

  public void controller_telemetry(){
//button telemetry
SmartDashboard.putNumber("Left X", m_driverController.getLeftX());
SmartDashboard.putNumber("Left Y", m_driverController.getLeftY());
SmartDashboard.putNumber("Right X", m_driverController.getRightX());
SmartDashboard.putNumber("Right Y", m_driverController.getRightY());

SmartDashboard.putBoolean("Button 0", m_driverController.button(0).getAsBoolean());
SmartDashboard.putBoolean("Button 1", m_driverController.button(1).getAsBoolean());
SmartDashboard.putBoolean("Button 2", m_driverController.button(2).getAsBoolean());
SmartDashboard.putBoolean("Button 3", m_driverController.button(3).getAsBoolean());
SmartDashboard.putBoolean("Button 4", m_driverController.button(4).getAsBoolean());
SmartDashboard.putBoolean("Button 5", m_driverController.button(5).getAsBoolean());
SmartDashboard.putBoolean("Button 6", m_driverController.button(6).getAsBoolean());
SmartDashboard.putBoolean("Button 7", m_driverController.button(7).getAsBoolean());
SmartDashboard.putBoolean("Button 8", m_driverController.button(8).getAsBoolean());
SmartDashboard.putBoolean("Button 9", m_driverController.button(9).getAsBoolean());
SmartDashboard.putBoolean("Button 10", m_driverController.button(10).getAsBoolean());
SmartDashboard.putBoolean("Button 11", m_driverController.button(11).getAsBoolean());
SmartDashboard.putBoolean("Button 12", m_driverController.button(12).getAsBoolean());

SmartDashboard.putBoolean("POV Center", m_driverController.povCenter().getAsBoolean());
SmartDashboard.putBoolean("POV Up",m_driverController.povUp().getAsBoolean());
SmartDashboard.putBoolean("POV Up Right",m_driverController.povUpRight().getAsBoolean());
SmartDashboard.putBoolean("POV Right",m_driverController.povRight().getAsBoolean());
SmartDashboard.putBoolean("POV Down Right",m_driverController.povDownRight().getAsBoolean());
SmartDashboard.putBoolean("POV Down",m_driverController.povDown().getAsBoolean());
SmartDashboard.putBoolean("POV Down Left",m_driverController.povDownLeft().getAsBoolean());
SmartDashboard.putBoolean("POV Left",m_driverController.povLeft().getAsBoolean());
SmartDashboard.putBoolean("POV Up Left",m_driverController.povUpLeft().getAsBoolean());

SmartDashboard.putData("Drivetrain", m_driveTrain);
SmartDashboard.putData("Power", m_pdp);

  }
}
