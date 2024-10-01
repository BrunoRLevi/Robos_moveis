/*
 * 
 * Aula 6 - Programa 3
 * 
*/
import lejos.nxt.LCD;

import lejos.nxt.UltrasonicSensor;
import lejos.nxt.SensorPort;
import lejos.nxt.Button;
import lejos.nxt.SensorPort;
import lejos.nxt.Motor;
import lejos.nxt.TouchSensor;
import lejos.robotics.navigation.*;

import lejos.util.Delay;


public class Ultrasonic {
  public static void main(String[] args) {
	TouchSensor toque = new TouchSensor(SensorPort.S3);
    UltrasonicSensor sonic = new UltrasonicSensor(SensorPort.S4);
    DifferentialPilot pilot = new DifferentialPilot(2.205 * 2.56, 4.527 * 2.56,        
                               Motor.A, Motor.B);

    while (true) {
        if(toque.isPressed())
            break;
    }

    Delay.msDelay(1000);

    while (!toque.isPressed()) {
      if (sonic.getDistance() > 30) {
        Motor.A.forward();
        Motor.B.forward();		
      } else {
        LCD.drawString("Encontrei obstaculo proximo", 0, 0);
        pilot.stop();
        Delay.msDelay(1500);
        
        pilot.arc(-30, -90, false);
          pilot.travel(30, false);
          break;
      }
    }  
  }
}

