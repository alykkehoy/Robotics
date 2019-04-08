//import lejos.robotics.RegulatedMotor;
//import lejos.robotics.SampleProvider;
//import lejos.utility.Delay;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;

import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.NXTUltrasonicSensor;

import lejos.robotics.subsumption.Arbitrator;
import lejos.robotics.subsumption.Behavior;

public class Lab03 {
	
	static NXTUltrasonicSensor ultrasonic_left = new NXTUltrasonicSensor(SensorPort.S4);
	static NXTUltrasonicSensor ultrasonic_right = new NXTUltrasonicSensor(SensorPort.S1);
	
	public static void main(String[] args) {
		Behavior b1 = new UltrasonicAvoid(ultrasonic_left, ultrasonic_right);
		Behavior b3 = new Roam();
		
		Behavior[] behaviors = {b3, b1};
		
		Arbitrator arbitrator = new Arbitrator(behaviors);
		arbitrator.go();
	}
}
