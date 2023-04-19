package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.arm.Claw;
import frc.robot.subsystems.arm.Claw.ClawStates;
import frc.robot.subsystems.arm.Wrist;
import frc.robot.subsystems.arm.Wrist.WristStates;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.Drivetrain.DrivetrainStates;
import frc.robot.subsystems.drivetrain.auto.AutoRoutines;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorStates;
import frc.robot.subsystems.intake.CubeShooter;
import frc.robot.subsystems.intake.CubeShooter.CubeShooterStates;
import frc.robot.subsystems.intake.CubeShooter.ShooterLevels;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeStates;
import frc.robot.subsystems.leds.LEDs;

public class RobotContainer {

	private final CommandXboxController m_driverController = new CommandXboxController(0);
	private final CommandXboxController m_operatorController = new CommandXboxController(1);
	// private final CommandXboxController m_testController = new
	// CommandXboxController(2);
	private final SendableChooser<Command> m_chooser = new SendableChooser<>();
	private final AutoRoutines m_routines = new AutoRoutines();
	private final Elevator m_elevator = Elevator.getInstance();
	private final LEDs m_leds = LEDs.getInstance();

	private final Wrist m_wrist = Wrist.getInstance();
	private final Claw m_claw = Claw.getInstance();
	private final Drivetrain m_drivetrain = Drivetrain.getInstance();
	private final Intake m_intake = Intake.getInstance();
	private final CubeShooter m_cubeShooter = CubeShooter.getInstance();

	public RobotContainer() {
		configureBindings();
		configureSendableChooser();
		m_leds.setDefaultCommand(m_leds.SetBreatheColor(new Color(0, 255, 0)));
		m_leds.getDefaultCommand().schedule();
	}

	private void configureBindings() {
		// #region Driver Controls
		m_drivetrain.setJoystickSuppliers(
				() -> m_driverController.getLeftY(),
				() -> m_driverController.getLeftX(),
				() -> m_driverController.getRightX());

		m_driverController.back().onTrue(m_drivetrain.zeroGyro());
		m_driverController.rightBumper()
				.onTrue(m_drivetrain.setState(DrivetrainStates.PRECISE))
				.onFalse(m_drivetrain.setState(DrivetrainStates.IDLE));

		// m_driverController.povUp()
		// .toggleOnTrue(m_drivetrain.setState(DrivetrainStates.SNAP_TO_ANGLE,
		// new HashMap<>(Map.of("Angle", 0d))))
		// .toggleOnFalse(m_drivetrain.setState(DrivetrainStates.IDLE));
		// m_driverController.povRight()
		// .onTrue(m_drivetrain.setState(DrivetrainStates.SNAP_TO_ANGLE,
		// new HashMap<>(Map.of("Angle", 270d))))
		// .onFalse(m_drivetrain.setState(DrivetrainStates.IDLE));
		// m_driverController.povDown()
		// .onTrue(m_drivetrain.setState(DrivetrainStates.SNAP_TO_ANGLE,
		// new HashMap<>(Map.of("Angle", 180d))))
		// .onFalse(m_drivetrain.setState(DrivetrainStates.IDLE));
		// m_driverController.povLeft()
		// .onTrue(m_drivetrain.setState(DrivetrainStates.SNAP_TO_ANGLE,
		// new HashMap<>(Map.of("Angle", 90d))))
		// .onFalse(m_drivetrain.setState(DrivetrainStates.IDLE));

		m_driverController.rightTrigger().onTrue(m_intake.setState(IntakeStates.INTAKE))
				.onFalse(m_intake.setState(IntakeStates.HOLDING));
		m_driverController.leftBumper().onTrue(m_intake.setState(IntakeStates.SPIT));
		m_driverController.x().onTrue(Commands.runOnce(() -> m_drivetrain.reseedModules()));
		m_driverController.povDown().onTrue(m_intake.setState(IntakeStates.RETRACTED));

		m_driverController.povLeft().onTrue(m_cubeShooter.setState(CubeShooterStates.CALIBRATE));
		m_driverController.povRight().onTrue(m_cubeShooter.setState(CubeShooterStates.SPIT))
				.onFalse(m_cubeShooter.setState(CubeShooterStates.RETRACTED));
		m_driverController.povUp().onTrue(m_cubeShooter.setState(CubeShooterStates.RETRACTED));
		m_driverController.b()
				.onTrue(m_cubeShooter.setState(CubeShooterStates.SCORE))
				.onFalse(m_cubeShooter.setState(CubeShooterStates.RETRACTED));
		m_driverController.leftTrigger()
				.onTrue(m_cubeShooter.setState(CubeShooterStates.INTAKE))
				.onFalse(m_cubeShooter.setState(CubeShooterStates.RETRACTED));
		// m_driverController.x().onTrue(m_drivetrain.setState(DrivetrainStates.SNAP_TO_ANGLE,
		// new HashMap<>(Map.of("Angle", 180.0))))
		// .onFalse(m_drivetrain.setState(DrivetrainStates.IDLE));
		m_driverController.y().onTrue(m_drivetrain.setState(DrivetrainStates.SNAP_TO_ANGLE,
				new HashMap<>(Map.of("Angle", 0.0))))
				.onFalse(m_drivetrain.setState(DrivetrainStates.IDLE));

		// Drop Game Piece
		m_driverController.a().onTrue(m_wrist.setState(WristStates.DROPPING)
				.andThen(new WaitCommand(0.15))
				.andThen(m_claw.setState(ClawStates.SCORE))
				.andThen(new WaitCommand(0.2))
				.andThen(m_elevator.setState(ElevatorStates.RETRACT))
				.andThen(new WaitCommand(0.5))
				.andThen(m_wrist.setState(WristStates.RETRACTED)));
		// #endregion

		// #region Operator Controls

		m_operatorController.back().whileTrue(m_leds.SetStaticColor(Color.kYellow));
		m_operatorController.start().whileTrue(m_leds.SetStaticColor(Color.kPurple));

		m_operatorController.povUp().onTrue(m_wrist.setState(WristStates.RETRACTED));
		m_operatorController.povDown().onTrue(m_wrist.setState(WristStates.DROPPING));

		m_operatorController.povRight()
				.onTrue(m_cubeShooter.Configure(ShooterLevels.MID))
				.whileTrue(m_leds.SetStaticColor(Color.kBlue));
		m_operatorController.povLeft()
				.onTrue(m_cubeShooter.Configure(ShooterLevels.HIGH))
				.whileTrue(m_leds.SetStaticColor(Color.kRed));

		m_operatorController.leftBumper().onTrue(
				Commands.sequence(
						m_claw.setState(ClawStates.OPEN),
						m_wrist.setState(WristStates.SUBSTATION)));

		m_operatorController.x().onTrue(m_claw.setState(ClawStates.OPEN));
		m_operatorController.y().onTrue(m_claw.setState(ClawStates.CLOSED));

		m_operatorController.rightTrigger().onTrue(
				Commands.sequence(
						m_elevator.setState(ElevatorStates.MID_CONE),
						Commands.waitSeconds(0.75),
						m_wrist.setState(WristStates.HALF_DUNK)));
		m_operatorController.rightBumper().onTrue(
				Commands.sequence(
						m_elevator.setState(ElevatorStates.TOP),
						Commands.waitSeconds(0.75),
						m_wrist.setState(WristStates.HALF_DUNK)));

		m_operatorController.a().onTrue(m_routines.HANDOFF());
		m_operatorController.b().onTrue(m_wrist.setState(WristStates.HALF_DUNK_HIGH));
		// #endregion

		// m_testController.leftBumper()
		// .onTrue(m_drivetrain.setState(DrivetrainStates.ON_THE_FLY_TRAJECTORY, new
		// HashMap<>(Map.of("Waypoint", 1))));
		// m_testController.a()
		// .onTrue(m_drivetrain.setState(DrivetrainStates.ON_THE_FLY_TRAJECTORY, new
		// HashMap<>(Map.of("Waypoint", 2))));
		// m_testController.b()
		// .onTrue(m_drivetrain.setState(DrivetrainStates.ON_THE_FLY_TRAJECTORY, new
		// HashMap<>(Map.of("Waypoint", 3))));
		// m_testController.x()
		// .onTrue(m_drivetrain.setState(DrivetrainStates.ON_THE_FLY_TRAJECTORY, new
		// HashMap<>(Map.of("Waypoint", 4))));
		// m_testController.y()
		// .onTrue(m_drivetrain.setState(DrivetrainStates.ON_THE_FLY_TRAJECTORY, new
		// HashMap<>(Map.of("Waypoint", 5))));
		// m_testController.povRight()
		// .onTrue(m_drivetrain.setState(DrivetrainStates.ON_THE_FLY_TRAJECTORY, new
		// HashMap<>(Map.of("Waypoint", 6))));
		// m_testController.povLeft()
		// .onTrue(m_drivetrain.setState(DrivetrainStates.ON_THE_FLY_TRAJECTORY, new
		// HashMap<>(Map.of("Waypoint", 7))));
		// m_testController.povDown()
		// .onTrue(m_drivetrain.setState(DrivetrainStates.ON_THE_FLY_TRAJECTORY, new
		// HashMap<>(Map.of("Waypoint", 8))));
		// m_testController.povUp()
		// .onTrue(m_drivetrain.setState(DrivetrainStates.ON_THE_FLY_TRAJECTORY, new
		// HashMap<>(Map.of("Waypoint", 9))));
		// m_testController.rightBumper()
		// .onTrue(m_drivetrain.setState(DrivetrainStates.ON_THE_FLY_TRAJECTORY, new
		// HashMap<>(Map.of("Waypoint", 10))));

		new Trigger(() -> m_cubeShooter.isIntaking).or(() -> m_intake.isIntaking)
				.whileTrue(m_leds.GamePieceIndicator(m_intake.backRollerOutputCurrent, m_cubeShooter.topRollerOutputCurrent));

	}

	private void configureSendableChooser() {
		m_chooser.setDefaultOption("Blank Auto", m_routines.BLANK_AUTO);
		m_chooser.addOption("Test", m_routines.TEST);
		m_chooser.addOption("High Cube/Mobility/Balance", m_routines.ONE_CONE_CHARGE_STATION);
		m_chooser.addOption("High Cone/Mobility/Balance", m_routines.ONE_CONE_CHARGE_STATION);
		m_chooser.addOption("2.5 Piece CLean Side", m_routines.ONE_CONE_ONE_CUBE);
		m_chooser.addOption("2 Bump Side", m_routines.ONE_CONE_ONE_CUBE_BUMP);
		m_chooser.addOption("2 Bump Side (Low)", m_routines.ONE_CONE_ONE_CUBE_LOW_BUMP);
		m_chooser.addOption("1.5 Bump Side", m_routines.ONE_CONE_PICKUP_CUBE_BUMP);
		m_chooser.addOption("High Cone Bump Side/Mobility", m_routines.ONE_CONE_BUMP);
		m_chooser.addOption("Two Cone Balance", m_routines.TWO_CONE_CHARGE_STATION);

		SmartDashboard.putData(m_chooser);
	}

	public Command getAutonomousCommand() {
		return m_chooser.getSelected();
	}

	// Useful for running pid controllers and motion profiles faster than the
	// default 20 ms loop time.
	public Runnable controlLoop() {
		return () -> {

		};
	}

	public void initLEDS() {
		m_leds.setDefaultCommand(m_leds.SetStaticColor(Color.kBlack));
		m_leds.getDefaultCommand().schedule();
	}

	// private Command substationToggle() {
	// return new ConditionalCommand(
	// (Command) new
	// SequentialCommandGroup(m_elevator.setState2(ElevatorStates.DOUBLE_SUBSTATION),
	// m_claw.setState(ClawStates.OPEN), m_arm.setState2(ArmStates.DROPPING)),
	// (Command) new SequentialCommandGroup(m_claw.setState(ClawStates.CLOSED),
	// new WaitCommand(1), new InstantCommand(() -> {
	// m_arm.setState(ArmStates.RETRACTED);
	// m_elevator.setState(ElevatorStates.RETRACT);
	// })),
	// () -> m_elevator.getState() != ElevatorStates.DOUBLE_SUBSTATION);
	// }
}
