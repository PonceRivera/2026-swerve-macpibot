package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Gancho2;
import frc.robot.subsystems.Intake2;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
//import frc.robot.subsystems.kraken;

public class RobotContainer {
        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        // Redujimos la tasa de giro de 0.75 a 0.45 para evitar sobrecarga en los
        // engranajes (tintineo)
        private double MaxAngularRate = RotationsPerSecond.of(0.45).in(RadiansPerSecond);

        // Multiplicador de velocidad de 0.2 (20%) a 1.0 (100%) ajustable por el piloto
        private double speedMultiplier = 1.0;

        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1)
                        .withRotationalDeadband(MaxAngularRate * 0.1)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final Telemetry logger = new Telemetry(MaxSpeed);
        private final CommandPS5Controller joystick = new CommandPS5Controller(0);
        private final CommandXboxController operator = new CommandXboxController(1);
        private final Joystick joystick2 = new Joystick(2);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        public final Intake2 intake = new Intake2();
        public final Shooter shooter = new Shooter();
        public final Limelight limelight = new Limelight();
        public final Gancho2 m_Gancho2 = new Gancho2();
        // public final kraken m_kraken = new kraken(); // Comentado por colisión de ID
        // 23 con el Shooter

        private final SendableChooser<Command> autoChooser;

        public RobotContainer() {
                registerNamedCommands();
                drivetrain.configureAutoBuilder();

                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Mode", autoChooser);

                configureBindings();
        }

        private void registerNamedCommands() {
                NamedCommands.registerCommand("BajarIntake", intake.mandarIntakeAPosicion(Intake2.Posicion.Abajo));
                NamedCommands.registerCommand("Subir", intake.mandarIntakeAPosicion(Intake2.Posicion.Home));
                NamedCommands.registerCommand("PararRoller", intake.pararRollerCommand());
                NamedCommands.registerCommand("Roller", intake.activarRollerCommand());
                NamedCommands.registerCommand("Shoot",
                                shooter.dispararSegunDistanciaCommand(limelight::getDistanciaMetros)
                                                .withTimeout(6.0));
                NamedCommands.registerCommand("GanchoReposo", m_Gancho2.mandarGanchoAPosicion(Gancho2.Posicion.Home));
                NamedCommands.registerCommand("GanchoSubir", m_Gancho2.mandarGanchoAPosicion(Gancho2.Posicion.Release));
                NamedCommands.registerCommand("GanchoBajar", m_Gancho2.mandarGanchoAPosicion(Gancho2.Posicion.Home));
                NamedCommands.registerCommand("CicloDefensaAuto", intake.cicloDefensaAutoCommand());
        }

        private void configureBindings() {

                drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> drive
                                .withVelocityX((MathUtil.applyDeadband(-joystick.getLeftY(), 0.1)
                                                + MathUtil.applyDeadband(-joystick2.getY(), 0.1)) * MaxSpeed
                                                * speedMultiplier)
                                .withVelocityY((MathUtil.applyDeadband(-joystick.getLeftX(), 0.1)
                                                + MathUtil.applyDeadband(-joystick2.getX(), 0.1)) * MaxSpeed
                                                * speedMultiplier)
                                .withRotationalRate((MathUtil.applyDeadband(-joystick.getRightX(), 0.1)
                                                + MathUtil.applyDeadband(-joystick2.getTwist(), 0.1)) * MaxAngularRate
                                                * speedMultiplier)));

                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                joystick.button(10).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
                joystick.cross().whileTrue(drivetrain.applyRequest(() -> brake));
                joystick.triangle().whileTrue(drivetrain.applyRequest(() -> point
                                .withModuleDirection(Rotation2d.kZero)));
                joystick.circle().whileTrue(drivetrain.applyRequest(() -> point
                                .withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

                // Control dinamico de velocidad del Swerve via D-Pad (povLeft / povRight)
                joystick.povLeft().onTrue(Commands.runOnce(() -> {
                        speedMultiplier = Math.min(1.0, speedMultiplier + 0.1);
                        SmartDashboard.putNumber("Swerve Speed %", speedMultiplier * 100);
                }));
                joystick.povRight().onTrue(Commands.runOnce(() -> {
                        speedMultiplier = Math.max(0.2, speedMultiplier - 0.1);
                        SmartDashboard.putNumber("Swerve Speed %", speedMultiplier * 100);
                }));

                // Auto-Align (Apuntar al AprilTag) con R1 (Boton 6) + Calibrar Shooter
                joystick.button(6).whileTrue(drivetrain.applyRequest(() -> drive
                                .withVelocityX(MathUtil.applyDeadband(joystick.getLeftY(), 0.2) * MaxSpeed
                                                * speedMultiplier)
                                .withVelocityY(MathUtil.applyDeadband(-joystick.getLeftX(), 0.2) * MaxSpeed
                                                * speedMultiplier)
                                .withRotationalRate(limelight.tieneObjetivo()
                                                ? -limelight.getXOffset() * 0.05 * MaxAngularRate
                                                : 0))
                                .alongWith(shooter.dispararSegunDistanciaCommand(limelight::getDistanciaMetros)));
                // .alongWith(m_kraken.runKrakenCommand())); // Ya controlado por el shooter

                joystick.button(8).whileTrue(shooter.shootCommand());

                joystick.button(7).whileTrue(shooter.dispararSegunDistanciaCommand(limelight::getDistanciaMetros));

                new JoystickButton(joystick2, 1).whileTrue(shooter.shootCommand());

                drivetrain.registerTelemetry(logger::telemeterize);
                operator.b().onTrue(intake.mandarIntakeAPosicion(Intake2.Posicion.Abajo));
                operator.a().onTrue(intake.mandarIntakeAPosicion(Intake2.Posicion.Home));
                operator.x().whileTrue(intake.activarRollerCommand());
                operator.y().whileTrue(intake.invertirRollerCommand());
                operator.leftBumper().whileTrue(m_Gancho2.subirManualCommand(0.6));
                operator.rightBumper().whileTrue(m_Gancho2.bajarManualCommand(0.6));

                // operator.povUp().onTrue(m_Gancho2.mandarGanchoAPosicion(Gancho2.Posicion.Release));
                // operator.povDown().onTrue(m_Gancho2.mandarGanchoAPosicion(Gancho2.Posicion.Home));

                // Default Commands for Manual Control (Operator Xbox Controller)
                // m_Gancho2.setDefaultCommand(m_Gancho2.controlLoopCommand(() ->
                // operator.getRightY()));
                // intake.setDefaultCommand(intake.controlLoopCommand(() ->
                // operator.getLeftY()));

                joystick.cross().onTrue(m_Gancho2.mandarGanchoAPosicion(Gancho2.Posicion.Home));
                joystick.circle().onTrue(m_Gancho2.mandarGanchoAPosicion(Gancho2.Posicion.Release));
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}

// Codigo final