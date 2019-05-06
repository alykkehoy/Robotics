//import lejos.robotics.RegulatedMotor;
//import lejos.robotics.SampleProvider;
//import lejos.utility.Delay;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;

import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.NXTUltrasonicSensor;


import lejos.robotics.geometry.Point;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.navigation.MovePilot;
import lejos.robotics.navigation.Pose;

public class Final {
	
	final static double goal_x = 7;
	final static double goal_y = 7;
	
	static double current_x = 0;
	static double current_y = 0;
	
	
//	distance between the 2 drive wheels from
//	the center point of the contact patches
	final static double b = .5;
	static double theta = Math.toRadians(90);
	
	
	static double motor_r_tacho_count = 0;
	static double motor_l_tacho_count = 0;

	private static NXTUltrasonicSensor ultrasonic_left = new NXTUltrasonicSensor(SensorPort.S1);
	private static NXTUltrasonicSensor ultrasonic_right = new NXTUltrasonicSensor(SensorPort.S2);
	
	private static EV3LargeRegulatedMotor motor_r = new EV3LargeRegulatedMotor(MotorPort.D);
	private static EV3LargeRegulatedMotor motor_l = new EV3LargeRegulatedMotor(MotorPort.A);
	
	public static void main(String[] args) {		
		motor_r.resetTachoCount();
		motor_l.resetTachoCount();
		
		double delta_s;
		double delta_theta;
		double s_r;
		double s_l;
		int loop_count = 0;

		
		while(current_x <= goal_x && current_y <= goal_y){			
			double delta_r_tacho = motor_r.getTachoCount() - motor_r_tacho_count; //Need to fix theese, still going neg
			double delta_l_tacho = motor_l.getTachoCount() - motor_l_tacho_count; //Need to fix theese, still going neg
			
			s_r = delta_r_tacho * 1;
			s_l = delta_l_tacho * 1;
			
			//System.out.print(delta_r_tacho);
			
			
			if(loop_count%100 == 0){
			System.out.println("Current X is: " + current_x);
			System.out.println("Current Y is: " + current_y);
			}
			
			
			delta_s = ((s_r + s_l) / 2);
			delta_theta = ((s_r - s_l) / b);

			double turn = Math.atan2(goal_y - current_y, goal_x - current_y);

			turn = turn * 25;
			
			motor_r.setSpeed(200 + (int)turn);
			motor_l.setSpeed(200 - (int)turn);
			
			motor_r.forward();
			motor_l.forward();

			
			double delta_x = delta_s * Math.cos(theta + (delta_theta / 2));
			double delta_y = delta_s * Math.sin(theta + (delta_theta / 2));			
		
			current_x += delta_x;
			current_y += delta_y;
			theta += delta_theta;
			
			motor_r_tacho_count = motor_r.getTachoCount();
			motor_l_tacho_count = motor_l.getTachoCount();
			
			loop_count++;
				
			}
		}
	}


