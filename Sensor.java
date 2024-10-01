import lejos.nxt.LCD;
import lejos.nxt.Button;
import lejos.nxt.Sound;
import lejos.util.Delay;
import lejos.nxt.Motor;
import lejos.robotics.navigation.*;
import lejos.nxt.SensorPort;
import lejos.nxt.TouchSensor;
public class Sensor {
  public static void main(String[] args) {
	Button.waitForAnyPress();
	LCD.clear();
	LCD.drawString("Sensor de Toque", 0, 0);
	TouchSensor toque = new TouchSensor(SensorPort.S3); //construtor do sensor de toque
	DifferentialPilot pilot = new DifferentialPilot(2.205 * 2.56, 4.527 * 2.56,        
                               Motor.A, Motor.B);
	pilot.travel(30, false);    
	pilot.arc(30, -90, true);
	while (!toque.isPressed() && pilot.isMoving()  ) { //&& pilot.isMoving() 
	}
	if (toque.isPressed()){
		Motor.A.stop();
		Motor.B.stop();
		Sound.playTone(5000, 400,100);
		LCD.drawString("Encostei em algo", 0, 0);
		Delay.msDelay(1500);

		//break;
	}
	pilot.arc(-30, -90, false);
	pilot.travel(30, false);


	// int i = 50;
	// while (true) {
	// 	if (toque.isPressed()){
	// 		Sound.playTone(i, 500,50);
	// 		Delay.msDelay(500);
	// 		LCD.clear();
	// 		LCD.drawString("tone " + i, 0, 0);
	// 		i += 10;
	// 	}
	// }
  }
}