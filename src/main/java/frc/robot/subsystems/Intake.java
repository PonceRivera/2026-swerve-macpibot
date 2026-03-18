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
    public static final double DEPLOY_BAJAR_SPEED = 0.15;
    public static final double DEPLOY_SUBIR_SPEED = -0.2;
    public static final double ROLLER_SPEED = 0.55;
    public static final double ROLLER_2_SPEED = 0.3;

    private double[] intakeMatrizPosicion = { 0.0, 0.5, 0.35 };
    private String[] intakePosicionesNombres = { "Reposo", "Abajo", "Defensa" };
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
                .p(1.0)
                .i(0.0)
                .d(0.0)
                .outputRange(-0.25, 0.25);

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
        if (index >= 0 && index < intakeMatrizPosicion.length) {
            activePositionName = intakePosicionesNombres[index];
            currentSetpoint = intakeMatrizPosicion[index];
            deployPidController.setReference(currentSetpoint, ControlType.kPosition);
        }
    }

    /** Mueve el brazo a la posición Home (Reposo). */
    public void irAHome() {
        irAIndice(0);
    }

    /** Mueve el brazo a la posición Abajo. */
    public void irAAbajo() {
        irAIndice(1);
    }

    /** Mueve el brazo a la posición Defensa. */
    public void irADefensa() {
        irAIndice(2);
    }

    /**
     * Va a un índice pero limita temporalmente la salida máxima del PID
     * para que vaya más lento.
     */
    public Command irAIndiceLimitadoCommand(int index, double maxOutput) {
        return runEnd(
                () -> {
                    double error = intakeMatrizPosicion[index] - deployEncoder.getPosition();
                    double output = error * 1.0; // kP
                    output = Math.max(-maxOutput, Math.min(maxOutput, output));
                    deployMotor.set(output);
                },
                () -> {
                    pararDeploy();
                });
    }

    public Command cicloDefensaAbajoCommand() {
        return runOnce(() -> rollerMotor2.set(ROLLER_2_SPEED)) // Arrancar roller 2
                .andThen(
                        run(() -> {
                            double error = intakeMatrizPosicion[2] - deployEncoder.getPosition();
                            double output = error * 1.0;
                            output = Math.max(-0.15, Math.min(0.15, output));
                            deployMotor.set(output);
                        })
                                .until(() -> Math.abs(intakeMatrizPosicion[2] - deployEncoder.getPosition()) < 0.05) // Hasta
                                .andThen(runOnce(this::pararDeploy)) // ¡FRENAR MOTOR AL LLEGAR!
                                .andThen(Commands.waitSeconds(0.5)) // Esperar medio segundo
                                .andThen(irAIndiceCommand(1))
                                .andThen(Commands.waitSeconds(0.5)))
                .repeatedly() // Repetir mientras se presiona el botón
                .finallyDo((interrupted) -> {
                    pararDeploy();
                    rollerMotor2.set(0); // Asegurar que el roller 2 se apague al soltar
                });
    }

    /** Detiene solo el motor del brazo. */
    public void pararDeploy() {
        deployMotor.set(0);
    }

    /**
     * Establece una velocidad personalizada para el brazo.
     * 
     * @param speed Velocidad (-1.0 a 1.0)
     */
    public void setDeploySpeed(double speed) {
        deployMotor.set(speed);
    }

    public void activarRoller() {
        rollerMotor.set(ROLLER_SPEED);
        rollerMotor2.set(ROLLER_2_SPEED);
    }

    /** Invierte el roller para sacar la pieza. */
    public void invertirRoller() {
        rollerMotor.set(-ROLLER_SPEED);
        rollerMotor2.set(-ROLLER_2_SPEED);
    }

    /** Detiene solo el roller. */
    public void pararRoller() {
        rollerMotor.set(0);
        rollerMotor2.set(0);
    }

    /**
     * Establece una velocidad personalizada para el roller.
     * 
     * @param speed Velocidad (-1.0 a 1.0)
     */
    public void setRollerSpeed(double speed) {
        rollerMotor.set(speed);
        rollerMotor2.set(speed != 0 ? ROLLER_2_SPEED * Math.signum(speed) : 0);
    }

    public void subirYApagar() {
        deployMotor.set(DEPLOY_SUBIR_SPEED);
        rollerMotor.set(0);
        rollerMotor2.set(0);
    }

    /** Detiene ambos motores por completo. */
    public void stop() {
        deployMotor.set(0);
        rollerMotor.set(0);
        rollerMotor2.set(0);
    }

    public Command bajarCommand() {
        return runEnd(this::bajar, this::pararDeploy);
    }

    /**
     * Sube el intake a velocidad configurable mientras se mantiene. Imprime
     * posición.
     */
    public Command subirManualCommand(double speed) {
        return runEnd(
                () -> {
                    deployMotor.set(-speed);
                    System.out.println("Intake Pos: " + deployEncoder.getPosition());
                },
                this::pararDeploy);
    }

    /**
     * Mientras se mantiene: sube el brazo y apaga el roller. Al soltar: para todo.
     */
    public Command subirYApagarCommand() {
        return runEnd(this::subirYApagar, this::stop);
    }

    /** Ejecuta una sola vez: para the roller sin mover el brazo. */
    public Command pararRollerCommand() {
        return runOnce(this::pararRoller);
    }

    /** Mientras se mantiene: activa solo el roller. Al soltar: lo detiene. */
    public Command activarRollerCommand() {
        return runEnd(this::activarRoller, this::pararRoller);
    }

    /**
     * Mientras se mantiene: activa el roller a una velocidad dada. Al soltar: lo
     * detiene.
     */
    public Command activarRollerVelocidadCommand(double speed) {
        return runEnd(() -> setRollerSpeed(speed), this::pararRoller);
    }

    /** Mientras se mantiene: invierte solo el roller. Al soltar: lo detiene. */
    public Command invertirRollerCommand() {
        return runEnd(this::invertirRoller, this::pararRoller);
    }

 
    public Command alimentarShooterCommand() {
        return runOnce(() -> setRollerSpeed(ROLLER_SPEED))
                .andThen(run(() -> setDeploySpeed(-0.3)).withTimeout(0.4))
                .andThen(run(() -> setDeploySpeed(0.3)).withTimeout(0.4))
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
            deployMotor.set(0.15); // Velocidad suave para buscar
            System.out.println("Intake Posicion Actual: " + deployEncoder.getPosition());
        })
                .beforeStarting(() -> System.out.println("Intake: Buscando posición..."))
                .finallyDo((interrupted) -> {
                    pararDeploy();
                    System.out.println("Intake: Posición final: " + deployEncoder.getPosition());
                });
    }

    public Command irAIndiceCommand(int index) {
        return runOnce(() -> {
            irAIndice(index);
            System.out.println("Intake: Moviendo a " + activePositionName);
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/Posicion Actual", deployEncoder.getPosition());
        SmartDashboard.putNumber("Intake/Objetivo (Setpoint)", currentSetpoint);
        SmartDashboard.putString("Intake/Nombre Posicion", activePositionName);

        intakeMatrizPosicion[0] = SmartDashboard.getNumber("Intake/Pos Reposo", intakeMatrizPosicion[0]);
        intakeMatrizPosicion[1] = SmartDashboard.getNumber("Intake/Pos Abajo", intakeMatrizPosicion[1]);
        intakeMatrizPosicion[2] = SmartDashboard.getNumber("Intake/Pos Defensa", intakeMatrizPosicion[2]);

        SmartDashboard.putNumber("Intake/Pos Reposo", intakeMatrizPosicion[0]);
        SmartDashboard.putNumber("Intake/Pos Abajo", intakeMatrizPosicion[1]);
        SmartDashboard.putNumber("Intake/Pos Defensa", intakeMatrizPosicion[2]);
    }
}