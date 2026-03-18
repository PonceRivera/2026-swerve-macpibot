// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Gancho;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
        private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

        /* Swerve requests */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final Telemetry logger = new Telemetry(MaxSpeed);

        /* Controllers */
        private final CommandXboxController joystick = new CommandXboxController(0);

        /* Subsystems */
        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        public final Intake intake = new Intake();
        public final Shooter shooter = new Shooter();
        public final Gancho gancho = new Gancho();

        private final SendableChooser<Command> autoChooser;

        public RobotContainer() {
                // 1. Registrar NamedCommands ANTES de configurar AutoBuilder
                registerNamedCommands();

                // 2. Configurar PathPlanner AutoBuilder con el drivetrain
                drivetrain.configureAutoBuilder();

                // 3. Crear el selector de autos y publicarlo en SmartDashboard
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Mode", autoChooser);

                configureBindings();
        }

        /**
         * Registra los comandos con nombre para PathPlanner.
         * Se deben registrar ANTES de buildAutoChooser().
         */
        private void registerNamedCommands() {

                // ── Intake ──────────────────────────────────────────────────────────
                // Baja el brazo (para correr en auto mientras el robot se
                // mueve)
                NamedCommands.registerCommand("BajarYActivarRoller", intake.bajarCommand());

                // Sube el brazo y apaga el roller
                NamedCommands.registerCommand("SubirYApagarRoller", intake.subirYApagarCommand());

                // Parar roller únicamente
                NamedCommands.registerCommand("PararRoller", intake.pararRollerCommand());

                // ── Shooter ─────────────────────────────────────────────────────────
                //NamedCommands.registerCommand("Shoot", shooter.shootFullCommand());

                // ── Gancho ──────────────────────────────────────────────────────────
                NamedCommands.registerCommand("GanchoReposo", gancho.irAIndiceCommand(0));

                NamedCommands.registerCommand("GanchoSubir", gancho.irAIndiceCommand(1));

                NamedCommands.registerCommand("GanchoBajar", gancho.irAIndiceCommand(2));
        }

        private void configureBindings() {

                drivetrain.setDefaultCommand(
                                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                                                .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                                                .withRotationalRate(-joystick.getRightX() * MaxAngularRate)));

                // Idle cuando el robot está disabled
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
                joystick.b().whileTrue(drivetrain.applyRequest(() -> point
                                .withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

                // LB: Invierte el roller para sacar piezas
                joystick.leftBumper().whileTrue(intake.invertirRollerCommand());

                drivetrain.registerTelemetry(logger::telemeterize);

                joystick.rightTrigger(0.1).whileTrue(shooter.shootCommand());
                joystick.rightTrigger(0.1).whileTrue(intake.cicloDefensaAbajoCommand());

                joystick.leftTrigger(0.1).whileTrue(intake.bajarCommand());

                joystick.rightBumper().onTrue(intake.irAIndiceCommand(2));

                joystick.povRight().whileTrue(intake.subirManualCommand(0.25));

                joystick.povLeft().onTrue(intake.irAIndiceCommand(0));

                joystick.x().whileTrue(intake.activarRollerCommand());

                joystick.povUp().whileTrue(gancho.subirManualCommand(0.3));

                joystick.povDown().whileTrue(gancho.bajarManualCommand(0.4));

                joystick.back().whileTrue(gancho.grabarDatosCommand());

                joystick.start().whileTrue(intake.grabarDatosCommand());
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}