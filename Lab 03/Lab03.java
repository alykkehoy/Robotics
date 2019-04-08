//import lejos.robotics.RegulatedMotor;
//import lejos.robotics.SampleProvider;
//import lejos.utility.Delay;
//import lejos.hardware.motor.EV3LargeRegulatedMotor;
//import lejos.hardware.port.MotorPort;
//import lejos.hardware.port.Port;

import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.NXTUltrasonicSensor;

import lejos.robotics.subsumption.Arbitrator;
import lejos.robotics.subsumption.Behavior;

public class Lab03 {
	
	static NXTUltrasonicSensor ultrasonic_left = new NXTUltrasonicSensor(SensorPort.S1);
	static NXTUltrasonicSensor ultrasonic_right = new NXTUltrasonicSensor(SensorPort.S2);
	
	public static void main(String[] args) {
//		EV3LargeRegulatedMotor motor_r = new EV3LargeRegulatedMotor(MotorPort.D);
//		EV3LargeRegulatedMotor motor_l = new EV3LargeRegulatedMotor(MotorPort.A);
//		motor_r.forward();
//		motor_l.forward();
//		
//		double motor_r_speed = 150;
//		double motor_l_speed = 150;
//		
//		final SampleProvider test_front = ultrasonic_front.getDistanceMode();
//		final SampleProvider test_side = ultrasonic_side.getDistanceMode();
//
//		float [] sample_front = new float[test_front.sampleSize()];
//		float [] sample_side = new float[test_side.sampleSize()];
//		
//		float distance_front = 0;
//		float distance_side = (float) 0.3;
//		
//		double kp = 200;
//		double ki = 0; 
//		double kd = 0;
//		
//		double integral_front = 0;
//		double integral_side = 0;
//		
//		float last_error_front = 0;
//		float last_error_side = 0;
//
//		while(true){			
//			test_front.fetchSample(sample_front, 0);
//			test_side.fetchSample(sample_side, 0);
//
//			double motor_r_turn = 0;
//			double motor_l_turn = 0;
//			
//			if(sample_front[0] != Float.POSITIVE_INFINITY){
//				float error_front = distance_front - sample_front[0];
//				float error_side = distance_side - sample_side[0];
//				
//				float derivative_front = error_front - last_error_front;
//				float derivative_side = error_side - last_error_side;
//				
//				integral_front = integral_front + error_front;
//				integral_side = integral_side + error_side;
//							
//				double correction_front = kp * error_front + ki * integral_front + kd * derivative_front;
//				double correction_side = kp * error_side + ki * integral_side + kd * derivative_side;
//	
//				motor_r_turn = -1 * correction_side;
//				motor_l_turn = correction_side;
//
//				last_error_front = error_front;
//				last_error_side = error_side;
//
//			}
//			
//			motor_r.setSpeed((int)(motor_r_speed + motor_r_turn));
//			motor_l.setSpeed((int)(motor_l_speed + motor_l_turn));
//			
//			System.out.println(motor_r_turn);
//		}
		
		Behavior b1 = new UltrasonicAvoid(ultrasonic_left, ultrasonic_right);
		
		Behavior[] behaviors = {b1};
		
		Arbitrator arbitrator = new Arbitrator(behaviors);
		arbitrator.go();
	}
}