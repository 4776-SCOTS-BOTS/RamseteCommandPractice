/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.
  private final CANSparkMax m_leftFrontMotor = new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless);
  private final CANSparkMax m_leftBackMotor = new CANSparkMax(DriveConstants.kLeftMotor2Port, MotorType.kBrushless);
  private final CANSparkMax m_rightFrontMotor = new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless);
  private final CANSparkMax m_rightBackMotor = new CANSparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushless);
  NetworkTableEntry m_xEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("X");
  NetworkTableEntry m_yEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Y");
  
  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftFrontMotor, m_rightFrontMotor);

  // The left-side drive encoder
  private final CANEncoder m_leftEncoder = m_leftFrontMotor.getEncoder();

  // The right-side drive encoder
  private final CANEncoder m_rightEncoder = m_rightFrontMotor.getEncoder();

  // The gyro sensor
  private final Gyro m_gyro = new ADXRS450_Gyro();

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    m_leftFrontMotor.restoreFactoryDefaults();
    m_leftBackMotor.restoreFactoryDefaults();
    m_rightFrontMotor.restoreFactoryDefaults();
    m_rightBackMotor.restoreFactoryDefaults();
    //Bind the front and back SparkMax's together using the follow() command
    m_leftBackMotor.follow(m_leftFrontMotor);
    m_rightBackMotor.follow(m_rightFrontMotor);

    // Sets the distance per pulse for the encoders
    m_leftEncoder.setPositionConversionFactor(DriveConstants.kRevolutionsToMeters);
    m_rightEncoder.setPositionConversionFactor(DriveConstants.kRevolutionsToMeters);
    m_leftEncoder.setVelocityConversionFactor(DriveConstants.kRPMtoMetersPerSecond);
    m_rightEncoder.setVelocityConversionFactor(DriveConstants.kRPMtoMetersPerSecond);
    System.out.println("pcr: "+m_rightEncoder.getPositionConversionFactor()+", VCF: "+m_rightEncoder.getVelocityConversionFactor());
    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  
    m_gyro.reset();

    m_leftFrontMotor.setIdleMode(IdleMode.kBrake);
    m_leftBackMotor.setIdleMode(IdleMode.kBrake);
    m_rightFrontMotor.setIdleMode(IdleMode.kBrake);
    m_rightBackMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_leftEncoder.getPosition(),
                      -m_rightEncoder.getPosition());
          
                      var translation = m_odometry.getPoseMeters().getTranslation();
                      m_xEntry.setNumber(translation.getX());
                      m_yEntry.setNumber(translation.getY());
    SmartDashboard.putString("Pose", getPose().toString());
    SmartDashboard.putString("Pose Rot", getPose().getRotation().toString());
    SmartDashboard.putString("Pose Dist", getPose().getTranslation().toString());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    //System.out.println("Speeds: "+m_leftEncoder.getVelocity()+", "+m_rightEncoder.getVelocity());
    //System.out.println("Current Pos("+getHeading()+"): "+m_leftEncoder.getPosition()+", "+m_rightEncoder.getPosition());
    //TODO: Look into this, getRate vs getVelocity: m_leftEncoder.getRate()
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), -m_rightEncoder.getVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  public void calculateArcadeDrive(double xSpeed, double rot) {
    var wheelSpeeds = DriveConstants.kDriveKinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    double leftFeedforward = DriveConstants.kFeedforward.calculate(speeds.leftMetersPerSecond);
    double rightFeedforward = DriveConstants.kFeedforward.calculate(speeds.rightMetersPerSecond);

    double leftOutput = DriveConstants.kLeftPIDController.calculate(m_leftEncoder.getVelocity(),
        speeds.leftMetersPerSecond);
    double rightOutput = DriveConstants.kRightPIDController.calculate(-m_rightEncoder.getVelocity(),
        speeds.rightMetersPerSecond);
        System.out.println("L: "+speeds.leftMetersPerSecond+", R: "+speeds.rightMetersPerSecond+
        ", FFL: "+leftFeedforward+", FFR: "+rightFeedforward+
        ", LV: "+m_leftEncoder.getVelocity()+", RV: "+m_rightEncoder.getVelocity()+
        ", LO: "+leftOutput+", RO: "+rightOutput);
    
    tankDriveVolts(leftOutput + leftFeedforward, rightOutput + rightFeedforward);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    String debug = debug(leftVolts, rightVolts);
    System.out.println(debug);
    m_leftFrontMotor.setVoltage(leftVolts);
    m_rightFrontMotor.setVoltage(-rightVolts);
    m_drive.feed();
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getPosition() + -m_rightEncoder.getPosition()) / 2.0;
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public String debug(double l, double r){
    SmartDashboard.putNumber("Gyro", m_gyro.getAngle());
    SmartDashboard.putNumber("OdoGyro", getHeading());
    SmartDashboard.putNumber("Left Position", m_leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Position", -m_rightEncoder.getPosition());
    SmartDashboard.putNumber("Left Velocity", m_leftEncoder.getVelocity());
    SmartDashboard.putNumber("Right Velocity", -m_rightEncoder.getVelocity());
    SmartDashboard.putNumber("Left Voltage", l);
    SmartDashboard.putNumber("Right Voltage", r);
    return String.format("Gyro(%f); OdoGyro(%f); Position(%f, %f); Velocity(%f, %f); Voltage(%f, %f);", 
      m_gyro.getAngle(), getHeading(), m_leftEncoder.getPosition(), -m_rightEncoder.getPosition(),
      m_leftEncoder.getVelocity(), -m_rightEncoder.getVelocity(), l, r
    );
  }
}
