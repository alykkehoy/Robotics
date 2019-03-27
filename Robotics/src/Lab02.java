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
		
		double motor_r_speed = 150;
		double motor_l_speed = 150;
		
		final SampleProvider test_front = ultrasonic_front.getDistanceMode();
		final SampleProvider test_side = ultrasonic_side.getDistanceMode();

		float [] sample_front = new float[test_front.sampleSize()];
		float [] sample_side = new float[test_side.sampleSize()];
		
		float distance_front = 0;
		float distance_side = (float) 0.3;
		
		double kp = 200;
		double ki = 0; 
		double kd = 0;
		
		double integral_front = 0;
		double integral_side = 0;
		
		float last_error_front = 0;
		float last_error_side = 0;

		while(true){			
			test_front.fetchSample(sample_front, 0);
			test_side.fetchSample(sample_side, 0);

			float error_front = distance_front - sample_front[0];
			float error_side = distance_side - sample_side[0];
			
			float derivative_front = error_front - last_error_front;
			float derivative_side = error_side - last_error_side;
			
			integral_front = integral_front + error_front;
			integral_side = integral_side + error_side;
			
			//Might not want to have front sensor on all three 
			//Start with just kp on front?
			
			double correction_front = kp * error_front + ki * integral_front + kd * derivative_front;
			double correction_side = kp * error_side + ki * integral_side + kd * derivative_side;
			
			

			double motor_r_turn = 20 - correction_side;
			double motor_l_turn = 20 + correction_side;
			
			motor_r.setSpeed((int)(motor_r_speed + motor_r_turn));
			motor_l.setSpeed((int)(motor_l_speed + motor_l_turn));
			
			last_error_front = error_front;
			last_error_side = error_side;
			
			System.out.println(correction_side);
		}
	}
	
	public float calc_p(double kp, float error){
		float p_val = (float)(kp * error);
		return p_val;
	}
	
	public float calc_i(double ki, float error){
		float i_val = 0;
		return i_val;
	}
	
	public float calc_d(double kd, float error){
		float d_val = 0;
		return d_val;
	}
}