// package frc.robot.subsystems;

// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.controls.DutyCycleOut;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.generated.TunerConstants;

// public class kraken extends SubsystemBase {
//     private final TalonFX motor;
//     private final DutyCycleOut request = new DutyCycleOut(0);
//     public static final double KRAKEN_SPEED = 0.5;

//     public kraken() {
//         // Inicializamos el motor con ID 23 en el bus CANivore definido en
//         // TunerConstants
//         motor = new TalonFX(23, TunerConstants.kCANBus.getName());
//     }

//     /**
//      * Activa el motor a una velocidad específica (-1.0 a 1.0).
//      */
//     public void setSpeed(double speed) {
//         motor.setControl(request.withOutput(speed));
//     }

//     /**
//      * Detiene el motor.
//      */
//     public void stop() {
//         motor.setControl(request.withOutput(0));
//     }

//     /**
//      * Comando para activar el motor mientras se mantenga presionado el botón.
//      */
//     public Command runKrakenCommand() {
//         return this.runEnd(
//             () -> setSpeed(KRAKEN_SPEED),
//             this::stop
//         );
//     }
// }
