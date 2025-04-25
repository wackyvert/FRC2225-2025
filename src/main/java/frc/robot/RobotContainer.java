// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.function.Supplier;

import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
   // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(1);
  final CommandJoystick driverJoystickL = new CommandJoystick(0);
  final CommandJoystick driverJoystickR = new CommandJoystick(2);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/falcon"));
  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the rotational velocity 
  // buttons are quick rotation positions to different ways to face
  // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
  AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                               OperatorConstants.LEFT_Y_DEADBAND),
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                               OperatorConstants.DEADBAND),
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                               OperatorConstants.RIGHT_X_DEADBAND),
                                                                 driverXbox.getHID()::getYButtonPressed,
                                                                 driverXbox.getHID()::getAButtonPressed,
                                                                 driverXbox.getHID()::getXButtonPressed,
                                                                 driverXbox.getHID()::getBButtonPressed);

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverJoystickR.getRawAxis(1)*-1 ,
                                                                () -> driverJoystickR.getRawAxis(0)*-1 )
                                                            .withControllerRotationAxis(()-> driverJoystickL.getRawAxis(0))
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverJoystickR::getX,
                                                                                             driverJoystickR::getY).withControllerRotationAxis(driverJoystickL::getX)
                                                                                             .headingWhile(true);


  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                   () -> -driverXbox.getLeftY(),
                                                                   () -> -driverXbox.getLeftX())
                                                               .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
                                                               .deadband(OperatorConstants.DEADBAND)
                                                               .scaleTranslation(0.8)
                                                               .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleSim     = driveAngularVelocitySim.copy()
                                                                     .withControllerHeadingAxis(() -> Math.sin(
                                                                                                    driverXbox.getRawAxis(
                                                                                                        2) * Math.PI) * (Math.PI * 2),
                                                                                                () -> Math.cos(
                                                                                                    driverXbox.getRawAxis(
                                                                                                        2) * Math.PI) *
                                                                                                      (Math.PI * 2))
                                                                     .headingWhile(true);

  Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);

  Command driveFieldOrientedAnglularVelocitySim = drivebase.driveFieldOriented(driveAngularVelocitySim);

  Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);

  Intake intakeSubsystem = new Intake();

  Elevator elevatorSubsystem = new Elevator(Constants.ELEVATOR_ID);

    LightSubsystem lights=new LightSubsystem();

    AlgaeIntake algaeIntakeSubsystem = new AlgaeIntake(lights);

 /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    NamedCommands.registerCommand("AlgaeIntakeGodAuto", new RunAlgaeIntakeGodAuto(algaeIntakeSubsystem).withTimeout(1.5));
      NamedCommands.registerCommand("lowerIntakeBit", new LowerIntakeAutoBit(intakeSubsystem).withTimeout(1));
      NamedCommands.registerCommand("elevatorDown", new LowerElevatorAuto(elevatorSubsystem).withTimeout(3));
  NamedCommands.registerCommand("elevatorUp", new RaiseElevatorAuto(elevatorSubsystem).withTimeout(3));
  NamedCommands.registerCommand("intakeDown", new LowerIntakeAutoBit(intakeSubsystem));
  NamedCommands.registerCommand("raiseIntake", new RaiseIntakeAuto(intakeSubsystem));
  NamedCommands.registerCommand("spitIntake2s", new OutIntake(intakeSubsystem).withTimeout(1.3));
  NamedCommands.registerCommand("spinIntake2s", new RunIntake(intakeSubsystem).withTimeout(2));
  NamedCommands.registerCommand("intakeAlgae2s", new RunAlgaeIntake (algaeIntakeSubsystem).withTimeout(2));
  NamedCommands.registerCommand("outtakeAlgae2s", new BackAlgaeIntake(algaeIntakeSubsystem).withTimeout(2));
  //autoChooser= AutoBuilder.buildAutoChooser("CenterSingleCoral");
  // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    //lights.setDefaultCommand(new LightsDefault(lights));
    // (Condition) ? Return-On-True : Return-on-False

    drivebase.setDefaultCommand(!RobotBase.isSimulation() ?
                                driveFieldOrientedAnglularVelocity :
                                driveFieldOrientedAnglularVelocitySim);


     driverJoystickR.button(1).whileTrue(Commands.runOnce(drivebase::zeroGyro, drivebase));
      //driverXbox.rightBumper().onTrue(Commands.none());
      driverXbox.a().whileTrue(new RunIntake(intakeSubsystem).repeatedly());
      driverXbox.b().whileTrue(new OutIntake(intakeSubsystem).repeatedly());
      driverXbox.x().whileTrue(new RunAlgaeIntake(algaeIntakeSubsystem).repeatedly());
      driverXbox.y().whileTrue(new BackAlgaeIntake(algaeIntakeSubsystem).repeatedly());
      driverXbox.rightTrigger().whileTrue(new RaiseIntake(intakeSubsystem));
      driverXbox.leftTrigger().whileTrue(new LowerIntake(intakeSubsystem));
      driverXbox.rightBumper().whileTrue(new RaiseElevator(elevatorSubsystem));
      driverXbox.leftBumper().whileTrue(new LowerElevator(elevatorSubsystem));
      driverJoystickL.button(1).whileTrue(new RaiseClimber(elevatorSubsystem));
    driverJoystickL.button(2).whileTrue(new LowerClimber(elevatorSubsystem));
   driverXbox.button(5).whileTrue(new DriveToPose(drivebase, reefClosestCenterFace()));
    //driverJoystickL.button(4).whileTrue(new DriveToPose(drivebase, nearestLeftCoral()));
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()

  {
       //NORMAL CENTER ONE CORAL
    return drivebase.getAutonomousCommand("CenterSingleCoral");
    //GOD AUTO
      // return drivebase.getAutonomousCommand("God Auto");
        //BLUE TOP RED BOTTOM
      //return drivebase.getAutonomousCommand("BlueTopRedBottomSingleCoral");
    //RED TOP BLUE BOTTOM
      //return drivebase.getAutonomousCommand("RedTopBlueBottomSingleCoral");

  }


  private Supplier<Pose2d> processorPose() {
    return () -> {
      if(AllianceFlipUtil.shouldFlip()){
        return FieldConstants.Processor.opposingCenterFace;
      }
      return FieldConstants.Processor.centerFace;
    };
  }
  private Supplier<Pose2d> reefClosestCenterFace() {
    Translation2d offset = new Translation2d(0.35, 0.10);
    return () -> {
      if(AllianceFlipUtil.shouldFlip()){

        Pose2d basePose = FieldConstants.Reef.centerFaces[drivebase.getSelectedReefInt()];
        System.out.println(drivebase.getSelectedReefInt()+"  "+ basePose.plus(new Transform2d(offset, new Rotation2d())));

        return AllianceFlipUtil.flip(basePose.plus(new Transform2d(offset, new Rotation2d())));
      }


      Pose2d basePose = FieldConstants.Reef.centerFaces[drivebase.getSelectedReefInt()];
      System.out.println(drivebase.getSelectedReefInt()+"  "+ basePose.plus(new Transform2d(offset, new Rotation2d())));

      return basePose.plus(new Transform2d(offset, new Rotation2d()));
    };
  }/*
  private Supplier<Pose2d> reefClosestCenterFace() {
    // Define your desired offset in the pose's local frame (e.g., forward 0.35m, left 0.10m)
    Translation2d offset = new Translation2d(0.35, 0.10); // adjust these values as needed

    return () -> {
      if(AllianceFlipUtil.shouldFlip()){
        Pose2d target = AllianceFlipUtil.flip(FieldConstants.Reef.centerFaces[0]);
        Pose2d offsetPose = target.plus(new Transform2d(offset, new Rotation2d()));
        System.out.println("Original: " + target + "\nOffset Pose: " + offsetPose + "\nCurrent Pose: " + drivebase.getPose());
        return AllianceFlipUtil.flip(offsetPose);
      }
      private Supplier<Pose2d> reefClosestCenterFace() {
        return () -> {
          Pose2d basePose = FieldConstants.Reef.centerFaces[drivebase.getSelectedReefInt()];
          Translation2d offset = new Translation2d(0.35, 0.10); // your fixed offset
          return basePose.plus(new Transform2d(offset, new Rotation2d()));
        };
      }

    };
  }
*/
  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
