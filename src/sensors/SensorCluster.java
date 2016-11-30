package sensors;

import utility.Formulas;
import utility.Log;
import utility.Trace;

public class SensorCluster {

	private int counter_;
	private double mean_;
	private double M2_;
	private Trace center_;
	
	public SensorCluster() {
		counter_ = 0;
		mean_ = 0.0;
		M2_ = 0.0;
	}
	
	public void initCenter(Trace trace) {
		center_ = new Trace(1);
		center_.copyTrace(trace);
	}
	
	
	public double distanceToCenter(Trace trace) {
		return Formulas.euclideanDistance(center_, trace);
	}
	
	public void addAccelerometer(Trace trace) {
		counter_++;
		double dist = this.distanceToCenter(trace);
		for(int j = 0; j < center_.dim; ++j) {
			center_.values[j] += (trace.values[j] - center_.values[j])/counter_; 
		}
		double delta = dist - mean_;
		mean_ += delta/counter_;
		M2_ += delta * (dist - mean_);
	}
	
	
	
}
