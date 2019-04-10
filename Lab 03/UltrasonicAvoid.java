//import lejos.utility.Delay;
//import lejos.hardware.port.MotorPort;
//import lejos.hardware.port.Port;
//import lejos.robotics.RegulatedMotor;
//import lejos.hardware.motor.EV3LargeRegulatedMotor;


import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.robotics.SampleProvider;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.hardware.sensor.NXTUltrasonicSensor;

import lejos.robotics.subsumption.Arbitrator;
import lejos.robotics.subsumption.Behavior;
import lejos.utility.Delay;

public class UltrasonicAvoid  implements Behavior{
	private DifferentialPilot pilot = new DifferentialPilot(4.32f, 12.2f, Motor.A, Motor.D);

	NXTUltrasonicSensor left;
	NXTUltrasonicSensor right;
	
	final float MAX_LEFT = 0.15f;
	final float MAX_RIGHT = 0.15f;
	
	boolean surpressed;

	final SampleProvider test_left;
	final SampleProvider test_right;
	float [] sample_left;
	float [] sample_right;

	
	boolean suppressed = false;
	
	public UltrasonicAvoid(NXTUltrasonicSensor left, NXTUltrasonicSensor right){
		this.left = left;
		this.right = right;
		
		this.test_left = left.getDistanceMode();
		this.test_right = right.getDistanceMode();
		this.sample_left = new float[test_left.sampleSize()];
		this.sample_right = new float[test_right.sampleSize()];

	}
	
	@Override
	public boolean takeControl() {
		test_left.fetchSample(sample_left, 0);
		test_right.fetchSample(sample_right, 0);

		return (sample_left[0] < MAX_LEFT || sample_right[0] < MAX_RIGHT);
	}

	@Override
	public void action() {
		System.out.println("UltrasonicAvoid");

		surpressed = false;
		Motor.A.setSpeed(200);
		Motor.D.setSpeed(200);
		
		while(!surpressed && sample_left[0] < MAX_LEFT){
			Motor.A.backward();
			Motor.D.forward();
			test_left.fetchSample(sample_left, 0);
		}
		
		while(!surpressed && sample_right[0] < MAX_RIGHT){
			Motor.D.backward();
			Motor.A.forward();
			test_right.fetchSample(sample_right, 0);
		}
		
		//Motor.A.forward();
		//Motor.D.forward();
		//Delay.msDelay(1000);
	}

	@Override
	public void suppress() {
		surpressed = true;
	}
}
