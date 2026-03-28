package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Intake2 extends SubsystemBase {

    public enum Posicion {
        Home,
        Abajo,
    }

    private double[] intakeMatrizPosicion = {
            0.0, 7.0,
    };

    private String[] intakePosicionesNombres = {
            "Home", "Abajo"
    }; 

    // Posiciones de los mecanismos
    private Posicion m_intake_current_position;
    // Setpoints de los mecanismos
    private double Intake_Objetivo = 0;
    // INPUTS sensores
    private double posicion_intake = 0;

    // Motores
    private SparkMax deployMotor = new SparkMax(13, MotorType.kBrushless);
    private SparkMax rollerMotor = new SparkMax(18, MotorType.kBrushless);
    private SparkMax rollerMotor2 = new SparkMax(14, MotorType.kBrushless);

    public static class Constants {
        public static final double intake_maxSetPoint = 7.5;
        public static final double intake_minSetPoint = 0.0;
        public static final double ROLLER_SPEED = 0.73;
        public static final double ROLLER_2_SPEED = 0.45;
    }

    // Encoders
    private RelativeEncoder deployEncoder = deployMotor.getEncoder();

    private ProfiledPIDController m_controller_intake = null;
    private TrapezoidProfile.Constraints constraintsIntake = null;

    public Intake2() {

        m_intake_current_position = Posicion.Home;

        SparkMaxConfig deployConfig = new SparkMaxConfig();
        constraintsIntake = new TrapezoidProfile.Constraints(5, 5); // Mas lento en aceleracion (era 10)
        m_controller_intake = new ProfiledPIDController(0.5, 0, 0, constraintsIntake); // P mas suave (era 1.0)

        deployConfig.inverted(false).idleMode(IdleMode.kBrake);
        deployConfig.smartCurrentLimit(20); // Limite de corriente reducido (era 40)

        deployMotor.configure(deployConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        rollerConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(30);
        rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rollerMotor2.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        ResetEncoders();
    }

    @Override
    public void periodic() {
        posicion_intake = deployEncoder.getPosition();
        SmartDashboard.putNumber("Encoder Intake", posicion_intake);
        SmartDashboard.putString(
                "Posicion Intake", intakePosicionesNombres[m_intake_current_position.ordinal()]);
    }

    private void ResetEncoders() {
        deployEncoder.setPosition(0); // Iniciamos en Home
    }

    public Command ResetIntake() {
        return runOnce(() -> deployEncoder.setPosition(0.0));
    }

    private void stopDeploy() {
        deployMotor.set(0);
    }

    private void controlLoop() {
        double intake_output = 0;
        double intake_setpoint = Intake_Objetivo;

        // Si estamos a menos de 0.05, paramos para evitar brincar la cadena (deadzone)
        if (Math.abs(posicion_intake - intake_setpoint) < 0.05) {
            stopDeploy();
            return;
        }

        m_controller_intake.setGoal(intake_setpoint);
        intake_output = m_controller_intake.calculate(posicion_intake);
        intake_output = Math.min(0.2, Math.max(-0.2, intake_output));

        deployMotor.set(intake_output);
    }

    private void setIntakeObjetivo(double nuevo_objetivo) {
        if (nuevo_objetivo > Constants.intake_maxSetPoint) {
            nuevo_objetivo = Constants.intake_maxSetPoint;
        } else if (nuevo_objetivo < Constants.intake_minSetPoint) {
            nuevo_objetivo = Constants.intake_minSetPoint;
        }
        Intake_Objetivo = nuevo_objetivo;
    }

    private void manualIntake(double input) {
        double scaledValue = (input + (input < 0 ? 0.25 : -0.25)) / (1 - 0.1);
        input = (Math.abs(input) > Math.abs(0.1)) ? scaledValue : 0;

        if (Math.abs(input) < 0.01) {
            return;
        }
        setIntakeObjetivo(posicion_intake + (input * 0.1));
    }

    public Command mandarIntakeAPosicion(Posicion posicion) {
        return run(() -> {
            m_intake_current_position = posicion;
            setIntakeObjetivo(intakeMatrizPosicion[posicion.ordinal()]);
            controlLoop();
        });
    }


    public Command controlLoopCommand(DoubleSupplier raw_intake) {
        return run(() -> {
            manualIntake(raw_intake.getAsDouble());
            controlLoop();
        }).finallyDo((interrupted) -> stopDeploy());
    }

    // --- Roller Methods ---

    public void activarRoller() {
        rollerMotor.set(Constants.ROLLER_SPEED);
        rollerMotor2.set(Constants.ROLLER_2_SPEED);
    }

    public void invertirRoller() {
        rollerMotor.set(-Constants.ROLLER_SPEED);
        rollerMotor2.set(-Constants.ROLLER_2_SPEED);
    }

    public void pararRoller() {
        rollerMotor.set(0);
        rollerMotor2.set(0);
    }

    // --- Roller Commands ---

    public Command activarRollerCommand() {
        return runEnd(this::activarRoller, this::pararRoller);
    }

    public Command invertirRollerCommand() {
        return runEnd(this::invertirRoller, this::pararRoller);
    }

    public Command pararRollerCommand() {
        return runOnce(this::pararRoller);
    }

    public Command bajarPorTiempoCommand(double segundos) {
        return runOnce(() -> mandarIntakeAPosicion(Posicion.Abajo)).andThen(Commands.waitSeconds(segundos));
    }

    public Command subirPorTiempoCommand(double segundos) {
        return runOnce(() -> mandarIntakeAPosicion(Posicion.Home)).andThen(Commands.waitSeconds(segundos));
    }

    public Command cicloDefensaAutoCommand() {
        return Commands.waitSeconds(1.0)
                .andThen(
                        runOnce(() -> rollerMotor2.set(0.3))
                                .andThen(
                                        subirPorTiempoCommand(1.0)
                                                .andThen(bajarPorTiempoCommand(1.0)))
                                .repeatedly())
                .finallyDo((interrupted) -> {
                    stopDeploy();
                    pararRoller();
                });
    }
}
