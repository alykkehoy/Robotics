import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.robotics.SampleProvider;
import lejos.hardware.sensor.NXTUltrasonicSensor;

import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.subsumption.Arbitrator;
import lejos.robotics.subsumption.Behavior;

public class Roam implements Behavior{
	boolean surpressed = false;
	private DifferentialPilot pilot = new DifferentialPilot(4.32f, 12.2f, Motor.A, Motor.D);
	public Roam(){
		
		
		
	}

	@Override
	public boolean takeControl() {
		// TODO Auto-generated method stub
		return true;
	}

	@Override
	public void action() {
		System.out.println("Roaming");
		surpressed = false;
		
		while (!surpressed){
			Motor.A.setSpeed(200);
			Motor.D.setSpeed(200);
			Motor.A.forward();
			Motor.D.forward();
		}
	}

	@Override
	public void suppress() {
		surpressed = true;
	}

}
