package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    public static final int DEPLOY_MOTOR_CAN_ID = 13;
    public static final int ROLLER_MOTOR_CAN_ID = 18;
    public static final int ROLLER_MOTOR_2_CAN_ID = 14;
    public static final double DEPLOY_BAJAR_SPEED = 0.07;
    public static final double DEPLOY_SUBIR_SPEED = -0.22;
    public static final double ROLLER_SPEED = 0.57;
    public static final double ROLLER_2_SPEED = 0.4;

    private static final double DEPLOY_KP = 1.0;
    private static final double POSITION_TOLERANCE = 0.005;
    private static final double FEED_SPEED = 0.3;
    private static final double FEED_DURATION = 0.4;

    private double[] posiciones = { 0.2, 2.5, 0.15 };
    private final String[] posicionesNombres = { "Reposo", "Abajo", "Defensa" };
    private double currentSetpoint = 0.0;
    private String activePositionName = "Reposo";

    private final SparkMax deployMotor = new SparkMax(DEPLOY_MOTOR_CAN_ID, MotorType.kBrushless);
    private final RelativeEncoder deployEncoder;
    private final SparkClosedLoopController deployPidController;
    private final SparkMax rollerMotor = new SparkMax(ROLLER_MOTOR_CAN_ID, MotorType.kBrushless);
    private final SparkMax rollerMotor2 = new SparkMax(ROLLER_MOTOR_2_CAN_ID, MotorType.kBrushless);

    public Intake() {
        SparkMaxConfig deployConfig = new SparkMaxConfig();
        deployConfig.idleMode(IdleMode.kBrake);
        deployConfig.smartCurrentLimit(40);
        deployConfig.closedLoop
                .p(1)
                .i(0)
                .d(0)
                .outputRange(-0.15, 0.15);

        deployMotor.configure(deployConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        deployEncoder = deployMotor.getEncoder();
        deployEncoder.setPosition(0.0);

        deployPidController = deployMotor.getClosedLoopController();

        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        rollerConfig.idleMode(IdleMode.kCoast);
        rollerConfig.smartCurrentLimit(30);
        rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig roller2Config = new SparkMaxConfig();
        roller2Config.idleMode(IdleMode.kCoast);
        roller2Config.smartCurrentLimit(30);
        rollerMotor2.configure(roller2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void bajar() {
        deployMotor.set(DEPLOY_BAJAR_SPEED);
    }

    public void subir() {
        deployMotor.set(DEPLOY_SUBIR_SPEED);
    }

    public void irAIndice(int index) {
        if (index < 0 || index >= posiciones.length) {
            return;
        }
        activePositionName = posicionesNombres[index];
        currentSetpoint = posiciones[index];
        deployPidController.setReference(currentSetpoint, ControlType.kPosition);
    }

    public void irAHome() {
        irAIndice(0);
    }

    public void irAAbajo() {
        irAIndice(1);
    }

    public void irADefensa() {
        irAIndice(2);
    }

    public void pararDeploy() {
        deployMotor.set(0);
    }

    public void setDeploySpeed(double speed) {
        deployMotor.set(speed);
    }

    private void moverLimitado(int index, double maxOutput) {
        double error = posiciones[index] - deployEncoder.getPosition();
        double output = error * DEPLOY_KP;
        output = Math.max(-maxOutput, Math.min(maxOutput, output));
        deployMotor.set(output);
    }

    private boolean llegueAPosicion(int index) {
        return Math.abs(posiciones[index] - deployEncoder.getPosition()) < POSITION_TOLERANCE;
    }

    // --- Métodos del roller ---

    public void activarRoller() {
        rollerMotor.set(ROLLER_SPEED);
        rollerMotor2.set(ROLLER_2_SPEED);
    }

    public void invertirRoller() {
        rollerMotor.set(-ROLLER_SPEED);
        rollerMotor2.set(-ROLLER_2_SPEED);
    }

    public void pararRoller() {
        rollerMotor.set(0);
        rollerMotor2.set(0);
    }

    public void setRollerSpeed(double speed) {
        rollerMotor.set(speed);
        rollerMotor2.set(speed != 0 ? ROLLER_2_SPEED * Math.signum(speed) : 0);
    }

    // --- Métodos combinados ---

    public void subirIntake() {
        subir();
    }

    public void stop() {
        pararDeploy();
        pararRoller();
    }

    // --- Commands ---

    public Command bajarCommand() {
        return runEnd(this::bajar, this::pararDeploy);
    }

    public Command subirManualCommand(double speed) {
        return runEnd(
                () -> {
                    deployMotor.set(DEPLOY_SUBIR_SPEED);
                    System.out.println("Intake Pos: " + deployEncoder.getPosition());
                },
                this::pararDeploy);
    }

    public Command subirCommand() {
        return runEnd(this::subirIntake, this::stop);
    }

    public Command pararRollerCommand() {
        return runOnce(this::pararRoller);
    }

    public Command activarRollerCommand() {
        return runEnd(this::activarRoller, this::pararRoller);
    }

    public Command activarRollerVelocidadCommand(double speed) {
        return runEnd(() -> setRollerSpeed(speed), this::pararRoller);
    }

    public Command invertirRollerCommand() {
        return runEnd(this::invertirRoller, this::pararRoller);
    }

    public Command irAIndiceLimitadoCommand(int index, double maxOutput) {
        return runEnd(
                () -> moverLimitado(index, maxOutput),
                this::pararDeploy);
    }

    public Command cicloDefensaAbajoCommand() {
        return runOnce(() -> rollerMotor2.set(ROLLER_2_SPEED))
                .andThen(
                        subirPorTiempoCommand(0.8)
                                .andThen(bajarPorTiempoCommand(1.3)))
                .repeatedly()
                .finallyDo((interrupted) -> {
                    stop();
                    subirPorTiempoCommand(0.8).schedule();
                });
    }

    public Command alimentarShooterCommand() {
        return runOnce(() -> setRollerSpeed(ROLLER_SPEED))
                .andThen(run(() -> setDeploySpeed(-FEED_SPEED)).withTimeout(FEED_DURATION))
                .andThen(run(() -> setDeploySpeed(FEED_SPEED)).withTimeout(FEED_DURATION))
                .finallyDo(this::stop);
    }

    public Command cicloDisparoCommand() {
        return runOnce(this::activarRoller)
                .andThen(runOnce(this::irAAbajo))
                .andThen(Commands.waitSeconds(3.0))
                .andThen(runOnce(this::irAHome))
                .andThen(Commands.waitSeconds(0.3))
                .repeatedly()
                .finallyDo((interrupted) -> stop());
    }

    public Command grabarDatosCommand() {
        return run(() -> {
            deployMotor.set(DEPLOY_BAJAR_SPEED);
            System.out.println("Intake Posicion Actual: " + deployEncoder.getPosition());
        })
                .beforeStarting(() -> System.out.println("Intake: Buscando posición..."))
                .finallyDo((interrupted) -> {
                    pararDeploy();
                    System.out.println("Intake: Posición final: " + deployEncoder.getPosition());
                });
    }

    public Command irAHomeLentoCommand() {
        return irAIndiceLimitadoCommand(0, 0.1);
    }

    public Command irAIndiceCommand(int index) {
        return runOnce(() -> {
            irAIndice(index);
            System.out.println("Intake: Moviendo a " + activePositionName);
        });
    }

    public Command bajarPorTiempoCommand(double segundos) {
        return bajarCommand().withTimeout(segundos);
    }

    public Command subirPorTiempoCommand(double segundos) {
        return subirCommand().withTimeout(segundos);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/Posicion Actual", deployEncoder.getPosition());
        SmartDashboard.putNumber("Intake/Objetivo (Setpoint)", currentSetpoint);
        SmartDashboard.putString("Intake/Nombre Posicion", activePositionName);

        for (int i = 0; i < posiciones.length; i++) {
            String key = "Intake/Pos " + posicionesNombres[i];
            posiciones[i] = SmartDashboard.getNumber(key, posiciones[i]);
            SmartDashboard.putNumber(key, posiciones[i]);
        }
    }
}