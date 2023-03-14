package frc.robot;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmStates;
import frc.robot.subsystems.arm.Claw;
import frc.robot.subsystems.arm.Claw.ClawStates;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.Drivetrain.DrivetrainStates;
import frc.robot.subsystems.drivetrain.auto.AutoRoutines;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorStates;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeStates;
import frc.robot.subsystems.leds.LEDs;

public class RobotContainer {

	private final CommandXboxController m_driverController = new CommandXboxController(0);
	private final CommandXboxController m_operatorController = new CommandXboxController(1);
	private final SendableChooser<Command> m_chooser = new SendableChooser<>();
	private final AutoRoutines m_routines = new AutoRoutines();
	private final Elevator m_elevator = Elevator.getInstance();
	private final LEDs m_leds = LEDs.getInstance();

	private final Arm m_arm = Arm.getInstance();
	private final Claw m_claw = Claw.getInstance();
	private final Drivetrain m_drivetrain = Drivetrain.getInstance();
	private final Intake m_intake = Intake.getInstance();

	public RobotContainer() {
		configureBindings();
		configureSendableChooser();
		m_leds.setDefaultCommand(m_leds.setBreatheColor(Constants.TEAM_COLOR));
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

		m_driverController.y()
				.onTrue(m_drivetrain.setState(DrivetrainStates.SNAP_TO_ANGLE,
						new HashMap<>(Map.of("Angle", 0d))))
				.onFalse(m_drivetrain.setState(DrivetrainStates.IDLE));
		m_driverController.b()
				.onTrue(m_drivetrain.setState(DrivetrainStates.SNAP_TO_ANGLE,
						new HashMap<>(Map.of("Angle", 270d))))
				.onFalse(m_drivetrain.setState(DrivetrainStates.IDLE));
		m_driverController.a()
				.onTrue(m_drivetrain.setState(DrivetrainStates.SNAP_TO_ANGLE,
						new HashMap<>(Map.of("Angle", 180d))))
				.onFalse(m_drivetrain.setState(DrivetrainStates.IDLE));
		m_driverController.x()
				.onTrue(m_drivetrain.setState(DrivetrainStates.SNAP_TO_ANGLE,
						new HashMap<>(Map.of("Angle", 90d))))
				.onFalse(m_drivetrain.setState(DrivetrainStates.IDLE));

		m_driverController.rightTrigger().onTrue(m_intake.setState(IntakeStates.INTAKE_CONE))
				.onFalse(m_intake.setState(IntakeStates.HANDOFF));
		m_driverController.leftTrigger().onTrue(m_intake.setState(IntakeStates.INTAKE_CUBE))
				.onFalse(m_intake.setState(IntakeStates.HANDOFF));
		m_driverController.leftBumper().onTrue(m_intake.setState(IntakeStates.SPIT_CONE));
		m_driverController.povDown().onTrue(m_intake.setState(IntakeStates.RETRACTED));
		m_driverController.start().onTrue(m_intake.setState(IntakeStates.CALIBRATE));
		// #endregion

		// #region Operator Controlls
		m_operatorController.povRight().whileTrue(m_leds.setLedColor(Color.kYellow));
		m_operatorController.povLeft().whileTrue(m_leds.setLedColor(Color.kPurple));

		m_operatorController.povUp().onTrue(new InstantCommand(() -> m_arm.setState(ArmStates.RETRACTED)));
		m_operatorController.povDown().onTrue(new InstantCommand(() -> m_arm.setState(ArmStates.DROPPING)));
		m_operatorController.leftBumper().onTrue(
				Commands.sequence(
						m_claw.setState(ClawStates.OPEN),
						Commands.runOnce(() -> m_arm.setState(ArmStates.SUBSTATION))));

		m_operatorController.x().onTrue(m_claw.setState(ClawStates.OPEN));
		m_operatorController.y().onTrue(m_claw.setState(ClawStates.CLOSED));

		m_operatorController.rightTrigger()
				.onTrue(new InstantCommand(() -> m_elevator.setState(ElevatorStates.MID_CONE)));
		m_operatorController.rightBumper()
				.onTrue(new InstantCommand(() -> m_elevator.setState(ElevatorStates.TOP)));

		// m_operatorController.start().onTrue(substationToggle());
		// Handoff Cone Sequence
		m_operatorController.a().onTrue(m_routines.HANDOFF());

		// Drop Game Piece
		m_operatorController.b().onTrue(new InstantCommand(() -> m_arm.setState(ArmStates.DROPPING))
				.andThen(new WaitCommand(0.5))
				.andThen(m_claw.setState(ClawStates.OPEN))
				.andThen(new WaitCommand(0.2))
				.andThen(() -> m_elevator.setState(ElevatorStates.RETRACT))
				.andThen(() -> m_arm.setState(ArmStates.RETRACTED)));
		// #endregion

	}

	private void configureSendableChooser() {
		m_chooser.setDefaultOption("Blank Auto", m_routines.BLANK_AUTO);
		m_chooser.addOption("Score Cone & Balance Charge Station", m_routines.SCORE_CONE_CHARGE_STATION_COMMUNITY);
		m_chooser.addOption("One Meter Test", m_routines.ONE_METER_TEST);
		m_chooser.addOption("One Piece + Charge Station", m_routines.ONE_PIECE_CHARGE_STATION);
		SmartDashboard.putData(m_chooser);
	}

	public Command getAutonomousCommand() {
		return m_chooser.getSelected();
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
