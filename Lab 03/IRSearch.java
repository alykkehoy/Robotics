import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.robotics.SampleProvider;

import lejos.robotics.subsumption.Behavior;
import lejos.hardware.sensor.EV3IRSensor;

public class IRSearch implements Behavior{
	EV3IRSensor IR_sensor;
	final SampleProvider test_IR_sensor;
	float[] sample_IR;

	boolean surpressed;
	
	public IRSearch(EV3IRSensor IR_sensor){
		this.IR_sensor = IR_sensor;
		this.test_IR_sensor = IR_sensor.getSeekMode();
		this.sample_IR = new float[test_IR_sensor.sampleSize()];

	}

	@Override
	public boolean takeControl() {
		test_IR_sensor.fetchSample(sample_IR, 0);
		return ((sample_IR[0] > 2 || sample_IR[0] < -2) && sample_IR[1] != Float.POSITIVE_INFINITY);
	}

	@Override
	public void action() {
		System.out.println("IR Search");
		
		surpressed = false;
		
		Motor.A.setSpeed(200);
		Motor.D.setSpeed(200);
		
		while(!surpressed && sample_IR[0] < -2){
			Motor.A.forward();
			Motor.D.backward();
			test_IR_sensor.fetchSample(sample_IR, 0);
		}
		
		while(!surpressed && sample_IR[0] > 2){
			Motor.D.forward();
			Motor.A.backward();
			test_IR_sensor.fetchSample(sample_IR, 0);
		}
	}

	@Override
	public void suppress() {
		surpressed = true;
	}

}