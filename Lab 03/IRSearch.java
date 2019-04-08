import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.robotics.SampleProvider;
import lejos.hardware.sensor.NXTUltrasonicSensor;

import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.subsumption.Arbitrator;
import lejos.robotics.subsumption.Behavior;
import lejos.hardware.sensor.EV3IRSensor;

public class IRSearch implements Behavior{
	private DifferentialPilot pilot = new DifferentialPilot(4.32f, 12.2f, Motor.A, Motor.D);
	EV3IRSensor IR_sensor = new EV3IRSensor(SensorPort.S2);
	final SampleProvider test_IR_sensor;
	float[] sample_IR;

	
	public IRSearch(){
		test_IR_sensor = IR_sensor.getSeekMode();
		sample_IR = new float[test_IR_sensor.sampleSize()];
	}

	@Override
	public boolean takeControl() {
		IR_sensor.fetchSample(sample_IR, 0);
		return (sample_IR[0] != Float.POSITIVE_INFINITY);
	}

	@Override
	public void action() {
		System.out.println("IR Search");
		System.out.println(sample_IR[0]);
		
		if(sample_IR[1] < 0){
			pilot.rotate(-90);
		}
		else if(sample_IR[1] > 0){
			pilot.rotate(90);
		}
		
	}

	@Override
	public void suppress() {
		// TODO Auto-generated method stub
		
	}

}
