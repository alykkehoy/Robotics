//import lejos.robotics.RegulatedMotor;
//import lejos.robotics.SampleProvider;
//import lejos.utility.Delay;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;

import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.NXTUltrasonicSensor;

import lejos.robotics.subsumption.Arbitrator;
import lejos.robotics.subsumption.Behavior;

public class Lab03 {
	
	static NXTUltrasonicSensor ultrasonic_left = new NXTUltrasonicSensor(SensorPort.S4);
	static NXTUltrasonicSensor ultrasonic_right = new NXTUltrasonicSensor(SensorPort.S1);
	static EV3IRSensor IR_sensor = new EV3IRSensor(SensorPort.S2);
	
	public static void main(String[] args) {
		Behavior b1 = new UltrasonicAvoid(ultrasonic_left, ultrasonic_right);
		Behavior b2 = new IRSearch(IR_sensor);
		Behavior b3 = new Roam(IR_sensor);
		
		Behavior[] behaviors = {b3, b2, b1};
		
		Arbitrator arbitrator = new Arbitrator(behaviors);
		arbitrator.go();
	}
}
