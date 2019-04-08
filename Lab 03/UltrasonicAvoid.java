//import lejos.utility.Delay;
//import lejos.hardware.port.MotorPort;
//import lejos.hardware.port.Port;
//import lejos.robotics.RegulatedMotor;
//import lejos.hardware.motor.EV3LargeRegulatedMotor;


import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.robotics.SampleProvider;
import lejos.hardware.sensor.NXTUltrasonicSensor;

import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.subsumption.Arbitrator;
import lejos.robotics.subsumption.Behavior;

public class UltrasonicAvoid  implements Behavior{
	private DifferentialPilot pilot = new DifferentialPilot(4.32f, 12.2f, Motor.A, Motor.D);
	NXTUltrasonicSensor left;
	NXTUltrasonicSensor right;

	final SampleProvider test_left;
	final SampleProvider test_right;
	float [] sample_left;
	float [] sample_right;

	
	boolean suppressed = false;
	
	public UltrasonicAvoid(NXTUltrasonicSensor left, NXTUltrasonicSensor right){
		this.left = left;
		this.right = right;
		
		test_left = left.getDistanceMode();
		test_right = right.getDistanceMode();
		float [] sample_left = new float[test_left.sampleSize()];
		float [] sample_right = new float[test_right.sampleSize()];

	}
	
	@Override
	public boolean takeControl() {
		test_left.fetchSample(sample_left, 0);
		test_right.fetchSample(sample_right, 0);

		return (sample_left[0] < 1 || sample_right[0] < 1);
	}

	@Override
	public void action() {
		System.out.println("UltrasonicAvoid");

		if(sample_left[0] < 1){
			System.out.println("left");
			pilot.rotate(90);
			pilot.travel(10);
		}
		else if(sample_right[0] < 1){
			System.out.println("right");
			pilot.rotate(-90);
			pilot.travel(10);			
		}
	}

	@Override
	public void suppress() {
		// TODO Auto-generated method stub
		suppressed = true;
	}
}
