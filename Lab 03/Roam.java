import java.io.File;

import lejos.hardware.Sound;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.robotics.SampleProvider;

import lejos.robotics.subsumption.Behavior;

public class Roam implements Behavior{
	EV3IRSensor IR_sensor;
	final SampleProvider test_IR_sensor;
	float[] sample_IR;

	boolean surpressed = false;
	
	public Roam(EV3IRSensor IR_sensor){	
		this.IR_sensor = IR_sensor;
		this.test_IR_sensor = IR_sensor.getSeekMode();
		this.sample_IR = new float[test_IR_sensor.sampleSize()];
	}

	@Override
	public boolean takeControl() {
		return true;
	}

	@Override
	public void action() {
		System.out.println("Roaming");
		surpressed = false;

		Motor.A.setSpeed(200);
		Motor.D.setSpeed(200);
		
		while (!surpressed){
			Motor.A.forward();
			Motor.D.forward();
			test_IR_sensor.fetchSample(sample_IR, 0);
			
			if(sample_IR[1] != 0 && sample_IR[1] != Float.POSITIVE_INFINITY && sample_IR[1] < 5){
				System.out.println(sample_IR[1]);
				Motor.D.stop();
				Motor.A.stop();
				Sound.beep();
				
				System.exit(0);
			}
		}
	}

	@Override
	public void suppress() {
		surpressed = true;
	}

}