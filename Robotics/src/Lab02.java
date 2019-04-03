package com.mydomain;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;
import lejos.hardware.sensor.NXTUltrasonicSensor;

public class Lab02 {
	
	private static NXTUltrasonicSensor ultrasonic_front = new NXTUltrasonicSensor(SensorPort.S1);
	private static NXTUltrasonicSensor ultrasonic_side = new NXTUltrasonicSensor(SensorPort.S2);
	
	public static void main(String[] args) {
		EV3LargeRegulatedMotor motor_r = new EV3LargeRegulatedMotor(MotorPort.D);
		EV3LargeRegulatedMotor motor_l = new EV3LargeRegulatedMotor(MotorPort.A);
		motor_r.forward();
		motor_l.forward();
		
		double motor_r_speed = 200;
		double motor_l_speed = 200;
		
		final SampleProvider test_front = ultrasonic_front.getDistanceMode();
		final SampleProvider test_side = ultrasonic_side.getDistanceMode();

		float [] sample_front = new float[test_front.sampleSize()];
		float [] sample_side = new float[test_side.sampleSize()];
		
		float distance_front = 0;
		float distance_side = (float) 0.3;
		
		double kp_side = 200;
		double ki_side = .5; 
		double kd_side = 200;
		
		double kp_front = 200;
		double ki_front = .5; 
		double kd_front = 200;
		
		double integral_front = 0;
		double integral_side = 0;
		
		float last_error_front = 0;
		float last_error_side = 0;

		while(true){			
			test_front.fetchSample(sample_front, 0);
			test_side.fetchSample(sample_side, 0);

			double motor_r_turn = 0;
			double motor_l_turn = 0;
			
			if(sample_side[0] != Float.POSITIVE_INFINITY){
					
				float error_side = distance_side - sample_side[0];
				
				float derivative_side = error_side - last_error_side;
				
				integral_side = integral_side + error_side;
							
				double correction_side = kp_side * error_side + ki_side * integral_side + kd_side * derivative_side;
	
				motor_r_turn += -1 * correction_side;
				motor_l_turn += correction_side;
				
				last_error_side = error_side;
			}

			System.out.println(sample_front[0]);

			if(sample_front[0] != Float.POSITIVE_INFINITY && sample_front[0] < 0.4){
				
				float error_front = distance_front - sample_front[0];
				
				float derivative_front = error_front - last_error_front;
				
				integral_front = integral_front + error_front;
							
				double correction_front = kp_front * error_front + ki_front * integral_front + kd_front * derivative_front;
				
	
				motor_r_turn += correction_front;
				motor_l_turn += -1 * correction_front;
				
				last_error_front = error_front;
			}

			
			motor_r.setSpeed((int)(motor_r_speed + motor_r_turn));
			motor_l.setSpeed((int)(motor_l_speed + motor_l_turn));
			
		}
	}
}