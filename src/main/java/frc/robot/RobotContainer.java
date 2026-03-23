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

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Gancho;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        // Redujimos la tasa de giro de 0.75 a 0.45 para evitar sobrecarga en los engranajes (tintineo)
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

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        public final Intake intake = new Intake();
        public final Shooter shooter = new Shooter();
        public final Limelight limelight = new Limelight();
        public final Gancho gancho = new Gancho();

        private final SendableChooser<Command> autoChooser;

        public RobotContainer() {
                registerNamedCommands();
                drivetrain.configureAutoBuilder();

                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Mode", autoChooser);

                configureBindings();
        }

        private void registerNamedCommands() {
                NamedCommands.registerCommand("BajarIntake", intake.bajarPorTiempoCommand(1.3));
                NamedCommands.registerCommand("Subir", intake.subirPorTiempoCommand(1.4));
                NamedCommands.registerCommand("PararRoller", intake.pararRollerCommand());
                NamedCommands.registerCommand("Roller", intake.runOnce(intake::activarRoller));
                NamedCommands.registerCommand("Shoot", shooter.shootCommand()
                                .alongWith(Commands.waitSeconds(2.0).andThen(intake.cicloDefensaAbajoCommand()))
                                .withTimeout(7.0));
                NamedCommands.registerCommand("GanchoReposo", gancho.irAIndiceCommand(0));
                NamedCommands.registerCommand("GanchoSubir", gancho.irAIndiceCommand(1));
                NamedCommands.registerCommand("GanchoBajar", gancho.irAIndiceCommand(2));
        }

        private void configureBindings() {
                drivetrain.setDefaultCommand(
                   drivetrain.applyRequest(() -> drive
                      .withVelocityX(MathUtil.applyDeadband(joystick.getLeftY(), 0.2) * MaxSpeed * speedMultiplier)
                      .withVelocityY(MathUtil.applyDeadband(joystick.getLeftX(), 0.2) * MaxSpeed * speedMultiplier)
                      .withRotationalRate(MathUtil.applyDeadband(-joystick.getRightX(), 0.5) * MaxAngularRate * speedMultiplier)));

                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                joystick.button(10).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
                joystick.cross().whileTrue(drivetrain.applyRequest(() -> brake));
                joystick.circle().whileTrue(drivetrain.applyRequest(() -> point
                        .withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

                // Control dinamico de velocidad del Swerve via D-Pad (povUp / povDown)
                joystick.povUp().onTrue(Commands.runOnce(() -> {
                        speedMultiplier = Math.min(1.0, speedMultiplier + 0.1);
                        SmartDashboard.putNumber("Swerve Speed %", speedMultiplier * 100);
                }));
                joystick.povDown().onTrue(Commands.runOnce(() -> {
                        speedMultiplier = Math.max(0.2, speedMultiplier - 0.1);
                        SmartDashboard.putNumber("Swerve Speed %", speedMultiplier * 100);
                }));

                // Auto-Align (Apuntar al AprilTag) con R1 (Boton 6) + Calibrar Shooter
                joystick.button(6).whileTrue(
                    drivetrain.applyRequest(() -> drive
                        .withVelocityX(MathUtil.applyDeadband(joystick.getLeftY(), 0.2) * MaxSpeed * speedMultiplier)
                        .withVelocityY(MathUtil.applyDeadband(joystick.getLeftX(), 0.2) * MaxSpeed * speedMultiplier)
                        .withRotationalRate(limelight.tieneObjetivo() ? -limelight.getXOffset() * 0.05 * MaxAngularRate : 0)
                    ).alongWith(shooter.dispararSegunDistanciaCommand(limelight::getDistanciaMetros))
                );

                joystick.button(8).whileTrue(shooter.shootCommand());
                joystick.button(8).whileTrue(
                        Commands.waitSeconds(2.0).andThen(intake.cicloDefensaAbajoCommand())
                                                .finallyDo((interrupted) -> intake.subirPorTiempoCommand(1.4)
                                                                .schedule()));

                joystick.button(7).whileTrue(shooter.dispararSegunDistanciaCommand(limelight::getDistanciaMetros));
                joystick.button(7).whileTrue(
                                Commands.waitSeconds(2.0).andThen(intake.cicloDefensaAbajoCommand())
                                                .finallyDo((interrupted) -> intake.subirPorTiempoCommand(1.4)
                                                                .schedule()));

                drivetrain.registerTelemetry(logger::telemeterize);
                operator.b().onTrue(intake.subirPorTiempoCommand(1.4));
                operator.a().onTrue(intake.bajarPorTiempoCommand(1.3));
                operator.x().whileTrue(intake.activarRollerCommand());
                operator.y().whileTrue(intake.invertirRollerCommand());
                operator.leftBumper().whileTrue(gancho.subirManualCommand(0.3));
                operator.rightBumper().whileTrue(gancho.bajarManualCommand(0.4));

                operator.povUp().onTrue(gancho.moverPorTiempoCommand(0.7, 1.0));
                operator.povDown().onTrue(gancho.moverPorTiempoCommand(-0.7, 1.0));
                operator.povLeft().whileTrue(intake.invertirRollerCommand());
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}