// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

 // Inicializa el motor Spark Max en el puerto CAN 1 y el PID Controller
    private final CANSparkMax mimotor1 = new CANSparkMax(1, MotorType.kBrushed);

    //Declara el encoder Through bore conectado DIRECTAMENTE al Spark
    SparkAbsoluteEncoder encoderLanzador = mimotor1.getAbsoluteEncoder();

    // Constantes PID para el controlador PID
    private final double kP = 0.0;
    private final double kI = 0.0;
    private final double kD = 0.0;

    //Declara el controlador PID con las constantes
    private final PIDController pid = new PIDController(kP, kI, kD);

    // Setpoint (punto de referencia) en grados para el lanzador
    private final double setpoint = 0.0; // Setpoint en grados para el lanzador

    //Crear Control para accionar el motor
    XboxController controller = new XboxController(0);

    //Mover arriba el offset (código más amigable)
    private final double offset = 0.0;

  @Override
  public void robotInit() {
    
      // Configura el offset inicial del encoder
      encoderLanzador.setZeroOffset(Units.degreesToRotations(offset)); // Offset inicial

      //Configura la posición del encoder, pasa de rotaciones a grados para no tener que escribir esta funcion cada vez que usen el encoder:
      // Units.rotationstodegrees()
      // Para pasar rotaciones a grados = rotaciones * 360 (por eso esta 360 en el factor)
      encoderLanzador.setPositionConversionFactor(360);

  }

  @Override
  public void robotPeriodic() {

    // Calcula la posición actual del encoder en grados
    double position = encoderLanzador.getPosition();

    // Muestra la posición del encoder en el SmartDashboard
    SmartDashboard.putNumber("Posición del Encoder", position);


   // Calcula la salida del PID basado en la posición actual y el setpoint
    double output = pid.calculate(position, setpoint);
   
    //Mueve el motor en base al PID
    if (controller.getAButton()) {
       mimotor1.set(output);
    }else{
      mimotor1.set(0);
    }

    //EN CASO DE Q NO FUNCIONE EL MÉTODO DE ARRIBA USAR ESTE
    double outputtest = pid.calculate(position, Units.degreesToRotations(setpoint));

    //PRESIONAR EN CASO DE Q EL OUTPUT NORMAL NO FUNCIONE
    if (controller.getBButton()) {
      mimotor1.set(outputtest);
    }else{
      mimotor1.set(0);
    }
    
  // Muestra la salida del PID en el SmartDashboard
    SmartDashboard.putNumber("PID output", output);


}


  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}


  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}