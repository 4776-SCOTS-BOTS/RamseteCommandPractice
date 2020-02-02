/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Paths;
import java.util.List;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Tools.Pair;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();

    // The driver's controller
    private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
        m_robotDrive.setDefaultCommand(
                // A split-stick arcade command, with forward/backward controlled by the left
                // hand, and turning controlled by the right.
                new RunCommand(() -> {
                    m_robotDrive.curvatureDrive(-deadzone(m_driverController.getY(GenericHID.Hand.kLeft)), 
                    deadzone(m_driverController.getX(GenericHID.Hand.kRight)),false,(left,right)->{
                        double turn = (left - right)/2;
                        double speed = right + (left-right)/2;
                        double xSpeed = m_speedLimiter.calculate(speed)
                                * DriveConstants.kMaxSpeedMetersPerSecond;
                        double rot = -m_rotLimiter.calculate(turn)
                                * DriveConstants.kMaxAngularSpeedRadiansPerSecond;
                        m_robotDrive.calculateArcadeDrive(xSpeed, rot);
                    });
            }, m_robotDrive));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     * @throws IOException
     */
    public Command getDefaultRamseteAutonomousCommand() throws IOException {
        var table = NetworkTableInstance.getDefault().getTable("troubleshooting");
        var leftReference = table.getEntry("left_reference");
        var leftMeasurement = table.getEntry("left_measurement");
        var rightReference = table.getEntry("right_reference");
        var rightMeasurement = table.getEntry("right_measurement");
        
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                       DriveConstants.kvVoltSecondsPerMeter,
                                       DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            10);
    var turn = new CentripetalAccelerationConstraint(0.1);
    
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);
            //.addConstraint(turn);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(1, 1),
            new Translation2d(2, -1)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config
    );
    Trajectory testTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            //new Translation2d(1, 0),
            new Translation2d(1, 0)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(1, 0, new Rotation2d(0)),
        // Pass config
        config
    );
    Trajectory nonJSONTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            //new Translation2d(1, 0),
            new Translation2d(0.75, 0.75)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(1.5, 1.5, new Rotation2d(0, 3)),//new Rotation2d(Math.PI / 2)),
        // Pass config
        config
    );

    //Part one
    String file = "one";
    Trajectory PART1jsonTrajectory = TrajectoryUtil.fromPathweaverJson(Paths.get(
        "/home/lvuser/deploy/output/"+file+".wpilib.json"));
    System.out.println("Loaded file "+file+".wpilib.json");
    //Part 2!!!!!
    String PART2file = "two";
    Trajectory PART2jsonTrajectory = TrajectoryUtil.fromPathweaverJson(Paths.get(
        "/home/lvuser/deploy/output/"+PART2file+".wpilib.json"));
    System.out.println("I ALSO::: Loaded file "+PART2file+".wpilib.json");
    /*
    //Transformations:
    Transform2d transform = m_robotDrive.getPose().minus(jsonTrajectory.getInitialPose());
    Transform2d transform2 = m_robotDrive.getPose().minus(PART2jsonTrajectory.getInitialPose());
    //Move the trajectories by the difference so that they start at the same position as the robot
    Trajectory newTrajectory = jsonTrajectory.transformBy(transform);
    Trajectory PART2trajectory = PART2jsonTrajectory.transformBy(transform2);
    */
    m_robotDrive.resetOdometry(PART1jsonTrajectory.getInitialPose());
    
    
    // Paste this variable in
    RamseteController disabledRamsete = new RamseteController() {
        @Override
        public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters,
                double angularVelocityRefRadiansPerSecond) {
            return new ChassisSpeeds(linearVelocityRefMeters, 0.0, angularVelocityRefRadiansPerSecond);
        }
    };
    PIDController leftController = new PIDController(DriveConstants.kPDriveVel, 0, 0);
    PIDController rightController = new PIDController(DriveConstants.kPDriveVel, 0, 0);
    RamseteCommand rammand = new RamseteCommand(
        PART1jsonTrajectory, 
        m_robotDrive::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        DriveConstants.kDriveKinematics,
        (leftVolts, rightVolts) -> {
            m_robotDrive.tankDriveVolts(leftVolts, rightVolts);
    
            leftMeasurement.setNumber(m_robotDrive.getWheelSpeeds().leftMetersPerSecond);
            leftReference.setNumber(leftController.getSetpoint());
    
            rightMeasurement.setNumber(m_robotDrive.getWheelSpeeds().rightMetersPerSecond);
            rightReference.setNumber(rightController.getSetpoint());
        },
        m_robotDrive
    );
    RamseteCommand ramseteCommand = new RamseteCommand(
        PART1jsonTrajectory,
        m_robotDrive::getPose,
        //disabledRamsete,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                   DriveConstants.kvVoltSecondsPerMeter,
                                   DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        m_robotDrive::getWheelSpeeds,
        leftController,
        rightController,
        //m_robotDrive::tankDriveVolts,
    (leftVolts, rightVolts) -> {
        m_robotDrive.tankDriveVolts(leftVolts, rightVolts);

        leftMeasurement.setNumber(m_robotDrive.getWheelSpeeds().leftMetersPerSecond);
        leftReference.setNumber(leftController.getSetpoint());

        rightMeasurement.setNumber(m_robotDrive.getWheelSpeeds().rightMetersPerSecond);
        rightReference.setNumber(rightController.getSetpoint());
    },
        m_robotDrive
    );
    
    RamseteCommand PART2ramseteCommand = new RamseteCommand(
        PART2jsonTrajectory,
        m_robotDrive::getPose,
        //disabledRamsete,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                   DriveConstants.kvVoltSecondsPerMeter,
                                   DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        m_robotDrive::getWheelSpeeds,
        leftController,
        rightController,
        //m_robotDrive::tankDriveVolts,
    (leftVolts, rightVolts) -> {
        m_robotDrive.tankDriveVolts(leftVolts, rightVolts);

        leftMeasurement.setNumber(m_robotDrive.getWheelSpeeds().leftMetersPerSecond);
        leftReference.setNumber(leftController.getSetpoint());

        rightMeasurement.setNumber(m_robotDrive.getWheelSpeeds().rightMetersPerSecond);
        rightReference.setNumber(rightController.getSetpoint());
    },
        m_robotDrive
    );

    // Run path following command, then stop at the end.
    Command wait = new WaitCommand(0).andThen(()->{
        m_robotDrive.resetOdometry(PART1jsonTrajectory.getInitialPose());
        System.out.println("P1 done. Me="+m_robotDrive.getPose()+", Part2Init="+PART1jsonTrajectory.getInitialPose()+".");
        
    });
    Command full = new SequentialCommandGroup(
        ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0)),
        wait,
        PART2ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0))
    );
    return full;
  }
  
  
  public Command getAutonomousCommand() throws IOException {
    return MultiRamseteCommands("one","two","three");
  }

  private SequentialCommandGroup MultiRamseteCommands(String... files) throws IOException {
    SequentialCommandGroup c = new SequentialCommandGroup();
    for (String s : files) {
        var data = EasyRamseteCommand(s);
        c.addCommands(data.getT1().beforeStarting(()->{
            System.out.println("Command \'"+s+"\' is now running at "+data.getT2().getInitialPose());
            m_robotDrive.resetOdometry(data.getT2().getInitialPose());
        }, m_robotDrive));
    }
    return c;
  }
  private Pair<Command, Trajectory> EasyRamseteCommand(String file) throws IOException {
    Trajectory jsonTrajectory = TrajectoryUtil.fromPathweaverJson(Paths.get(
        "/home/lvuser/deploy/output/"+file+".wpilib.json"));
    System.out.println("EasyRamseteCommand loaded \'"+file+"\' successfully.");
    return new Pair<Command, Trajectory>(new RamseteCommand(
        jsonTrajectory,
        m_robotDrive::getPose,
        //disabledRamsete,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                   DriveConstants.kvVoltSecondsPerMeter,
                                   DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        m_robotDrive::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        m_robotDrive::tankDriveVolts,
        m_robotDrive
    ).andThen(() -> m_robotDrive.tankDriveVolts(0, 0)),jsonTrajectory);
  }
}
