// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.PhysicalConstants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private final WPI_TalonSRX frontLeftDrive = new WPI_TalonSRX(DrivetrainConstants.kFrontLeftMotorID);
  private final WPI_TalonSRX frontRightDrive = new WPI_TalonSRX(DrivetrainConstants.kFrontRightMotorID);

  private final VictorSPX backLeftDrive = new VictorSPX(DrivetrainConstants.kBackLeftMotorID);
  private final VictorSPX backRightDrive = new VictorSPX(DrivetrainConstants.kBackRightMotorID);

  private final DifferentialDrive differentialDrive = new DifferentialDrive(frontLeftDrive, frontRightDrive);

  private final Encoder frontLeftEncoder = new Encoder(DrivetrainConstants.kFrontLeftEncoderA, DrivetrainConstants.kFrontLeftEncoderB, false, EncodingType.k4X);
  private final Encoder backLeftEncoder = new Encoder(DrivetrainConstants.kBackLeftEncoderA, DrivetrainConstants.kBackLeftEncoderB, false, EncodingType.k4X);
  private final Encoder frontRightEncoder = new Encoder(DrivetrainConstants.kFrontRightEncoderA, DrivetrainConstants.kFrontRightEncoderB, false, EncodingType.k4X);
  private final Encoder backRightEncoder = new Encoder(DrivetrainConstants.kBackRightEncoderA, DrivetrainConstants.kBackRightEncoderB, false, EncodingType.k4X);   


  private final AHRS gyro;


  public Drivetrain() {
    //left side
    frontLeftDrive.configFactoryDefault();
    backLeftDrive.configFactoryDefault();

    frontLeftDrive.setNeutralMode(NeutralMode.Brake);
    backLeftDrive.setNeutralMode(NeutralMode.Brake);

    frontLeftDrive.setInverted(DrivetrainConstants.kFrontLeftMotorInverted);
    backLeftDrive.setInverted(DrivetrainConstants.kBackLeftMotorInverted);

    backLeftDrive.follow(frontLeftDrive);
 
    //right side
    frontRightDrive.configFactoryDefault();
    backRightDrive.configFactoryDefault();

    frontRightDrive.setNeutralMode(NeutralMode.Brake);
    backRightDrive.setNeutralMode(NeutralMode.Brake);

    frontRightDrive.setInverted(DrivetrainConstants.kFrontRightMotorInverted);
    backRightDrive.setInverted(DrivetrainConstants.kBackRightMotorInverted);

    backRightDrive.follow(frontRightDrive);

    //gyro
    gyro = new AHRS();
    
    new Thread(() -> {
      try {
          Thread.sleep(1000);
          zeroHeading();
      } catch (Exception e) {
      }
  }).start();
  

    frontLeftEncoder.setDistancePerPulse(PhysicalConstants.kDistancePerPulse);
    frontRightEncoder.setDistancePerPulse(PhysicalConstants.kDistancePerPulse);


  }

  @Override
  public void periodic() {
   telemetry();




  }



    /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    differentialDrive.arcadeDrive(fwd, rot);
  }
  

  // Tank style drive
  public void tankDrive(double leftSpeed, double rightSpeed) {
    differentialDrive.tankDrive(leftSpeed, rightSpeed);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    frontLeftEncoder.reset();
    frontRightEncoder.reset();
  }

  public double getAverageEncoderDistance() {
    return (frontLeftEncoder.getDistance() + frontRightEncoder.getDistance()) / 2.0;
  }

  public Encoder getLeftEncoder(){
    return frontLeftEncoder;
  }

  public Encoder getRightEncoder(){
    return frontRightEncoder;
  }


    /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    differentialDrive.setMaxOutput(maxOutput);
  }


  //TODO: look into this
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    // Publish encoder distances to telemetry.
    builder.addDoubleProperty("leftDistance", frontLeftEncoder::getDistance, null);
    builder.addDoubleProperty("rightDistance", frontRightEncoder::getDistance, null);
  }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
      gyro.reset();
    }

      /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360) * (DrivetrainConstants.kGyroReversed ? -1.0 : 1.0);
  }

 
  public double getTurnRate() {
    return gyro.getRate() * (DrivetrainConstants.kGyroReversed ? -1.0 : 1.0); //rate in degrees per second
  }

  public void telemetry(){
 // Update telemetry 
 SmartDashboard.putNumber("Left Encoder Raw", frontLeftEncoder.getRaw());
 SmartDashboard.putNumber("Left Encoder Distance", frontLeftEncoder.getDistance());
 SmartDashboard.putNumber("Left Encoder Rate", frontLeftEncoder.getRate());
 SmartDashboard.putBoolean("Left Encoder Forward", frontLeftEncoder.getDirection()); //is this useful?

 SmartDashboard.putNumber("Right Encoder Raw", frontRightEncoder.getRaw());
 SmartDashboard.putNumber("Right Encoder Distance", frontRightEncoder.getDistance());
 SmartDashboard.putNumber("Right Encoder Rate", frontRightEncoder.getRate());
 SmartDashboard.putBoolean("Right Encoder Forward", frontRightEncoder.getDirection()); //is this useful?

 SmartDashboard.putNumber("Back Left Encoder Raw", backLeftEncoder.getRaw());
 SmartDashboard.putNumber("Back Left Encoder Distance", backLeftEncoder.getDistance());
 SmartDashboard.putNumber("Back Left Encoder Rate", backLeftEncoder.getRate());
 SmartDashboard.putBoolean("Back Left Encoder Forward", backLeftEncoder.getDirection()); //is this useful?

 SmartDashboard.putNumber("Back Right Encoder Raw", backRightEncoder.getRaw());
 SmartDashboard.putNumber("Back Right Encoder Distance", backRightEncoder.getDistance());
 SmartDashboard.putNumber("Back Right Encoder Rate", backRightEncoder.getRate());
 SmartDashboard.putBoolean("Back Right Encoder Forward", backRightEncoder.getDirection()); //is this useful?

 SmartDashboard.putNumber("Average Distance", getAverageEncoderDistance());

 SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
 SmartDashboard.putNumber("Gyro Rotation Rate", gyro.getRate());
 SmartDashboard.putNumber("Gyro Compass Heading", gyro.getCompassHeading());
 SmartDashboard.putNumber("Gyro Altitude", gyro.getAltitude());
 SmartDashboard.putNumber("Gyro Displacement X", gyro.getDisplacementX());
 SmartDashboard.putNumber("Gyro Displacement Y", gyro.getDisplacementY());
 SmartDashboard.putNumber("Gyro Displacement Z", gyro.getDisplacementZ());
 SmartDashboard.putNumber("Gyro Pitch", gyro.getPitch());
 SmartDashboard.putNumber("Gyro Roll", gyro.getRoll());
 SmartDashboard.putNumber("Gyro Yaw", gyro.getYaw());
 SmartDashboard.putNumber("Gyro Velocity X", gyro.getVelocityX());
 SmartDashboard.putNumber("Gyro Velocity Y", gyro.getVelocityY());
 SmartDashboard.putNumber("Gyro Velocity Z", gyro.getVelocityZ());

 SmartDashboard.putNumber("Turn Rate", getTurnRate());
 SmartDashboard.putNumber("Heading", getHeading());

 SmartDashboard.putNumber("Front Left Motor Output Percentage", frontLeftDrive.getMotorOutputPercent());
 SmartDashboard.putNumber("Front Left Motor Output Voltage", frontLeftDrive.getMotorOutputVoltage());
 SmartDashboard.putNumber("Front Right Motor Output Percentage", frontRightDrive.getMotorOutputPercent());
 SmartDashboard.putNumber("Front Right Motor Output Percentage", frontRightDrive.getMotorOutputPercent());
  }
}
