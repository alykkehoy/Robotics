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
	
	final static double goal_x = 0;
	final static double goal_y = 100;
	
	static double current_x = 0;
	static double current_y = 0;
	
	
	static float [] sample_left;
	static float [] sample_right;
	
//	distance between the 2 drive wheels from
//	the center point of the contact patches
	final static double b = 20; //cms
	static double theta = 1.5708; //1.5708
	
	
	static double motor_r_tacho_count = 0;
	static double motor_l_tacho_count = 0;
	
	private static EV3LargeRegulatedMotor motor_r = new EV3LargeRegulatedMotor(MotorPort.A);
	private static EV3LargeRegulatedMotor motor_l = new EV3LargeRegulatedMotor(MotorPort.D);
	
	public static void main(String[] args) {
		float[] left_sample = new float[3];
		float[] right_sample = new float[3];
		
		Scan scan = new Scan(left_sample, right_sample);
		new Thread(scan).run();
		
		motor_r.resetTachoCount();
		motor_l.resetTachoCount();
		
		double delta_s;
		double delta_theta;
		double s_r;
		double s_l;
		int loop_count = 0;
		
		double constant_tacho = (Math.PI * 3.4) / 360;

		
		while(current_x <= goal_x && current_y <= goal_y){			
			double delta_r_tacho = motor_r.getTachoCount() - motor_r_tacho_count;
			double delta_l_tacho = motor_l.getTachoCount() - motor_l_tacho_count;
			
			
			motor_r_tacho_count = motor_r.getTachoCount();
			motor_l_tacho_count = motor_l.getTachoCount();
			
			s_r = delta_r_tacho * constant_tacho;
			s_l = delta_l_tacho * constant_tacho;
						
			if(loop_count%10000 == 0){
				System.out.println("Current X is: " + current_x);
				System.out.println("Current Y is: " + current_y);
				System.out.println("Current theta is: " + theta);
			}
			
			
			delta_s = ((s_r + s_l) / 2);
			delta_theta = ((s_r - s_l) / b);

			double turn = Math.atan2(goal_y - current_y, goal_x - current_y);
			double output;
			
			double error = turn - theta;

			output = error * 25; //25 is kp
			
			float sum = -10 * left_sample[0] + -5 * left_sample[1] + -1 * left_sample[2]
					  + 10 * right_sample[0] + 5 * right_sample[1] + 1 * right_sample[2];
			
			System.out.println(sample_left[0]);
			
			motor_r.setSpeed(200 + (int)output + (int)sum);
			motor_l.setSpeed(200 - (int)output - (int)sum);
			
			motor_r.forward();
			motor_l.forward();

			
			double delta_x = delta_s * Math.cos(theta + (delta_theta / 2));
			double delta_y = delta_s * Math.sin(theta + (delta_theta / 2));			
		
			current_x += delta_x;
			current_y += delta_y;
			theta += delta_theta;
			
			loop_count++;
			}
		}
	}



