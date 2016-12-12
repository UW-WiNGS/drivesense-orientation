package sensors;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import utility.Formulas;
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
	
	
	////===============static methods ====================================
	
	/**
	 * 
	 * @param cluster
	 * @param size
	 * @return
	 */
	
	public static List<Trace> slidingVariance(List<Trace> cluster, int size) {
		List<Trace> res = new ArrayList<Trace>();
		List<Trace> slidingWindow = new LinkedList<Trace>();
		for(int i = 0; i < cluster.size(); ++i) {
			Trace cur = cluster.get(i);
			slidingWindow.add(cur);
			if(slidingWindow.size() == size) {
				double variance = SensorCluster.calculateClusterVariance(slidingWindow);
				slidingWindow.remove(0);
				Trace ntr = new Trace(1);
				ntr.time = cur.time;
				ntr.values[0] = variance;
				res.add(ntr);
			}
		}
		return res;
	}
	
	
	
	public static double calculateClusterVariance(List<Trace> cluster) {
		double mean = 0.0, M2 = 0.0;
		Trace center = new Trace(3);
		center.copyTrace(cluster.get(0));
		int counter = 0;
		for(int i = 1; i < cluster.size(); ++i) {
			Trace cur = cluster.get(i);
			double dist = 0.0;
			dist = Formulas.euclideanDistance(center, cur);
			counter ++;
			for(int j = 0; j < center.dim; ++j) {
				center.values[j] += (cur.values[j] - center.values[j])/counter; 
			}
			double delta = dist - mean;
			mean += delta/counter;
			M2 += delta * (dist - mean);
		}
		return M2/counter;
	}
	
	public static List<Trace> trackClusterVariance(List<Trace> cluster) {
		List<Trace> res = new ArrayList<Trace>();
		Trace center = new Trace(3);
		center.copyTrace(cluster.get(0));
		int counter = 0;
		double mean = 0.0, M2 = 0.0;
		for(int i = 1; i < cluster.size(); ++i) {
			Trace cur = cluster.get(i);
			double dist = 0.0;
			dist = Formulas.euclideanDistance(center, cur);
			counter ++;
			for(int j = 0; j < center.dim; ++j) {
				center.values[j] += (cur.values[j] - center.values[j])/counter; 
			}
			double delta = dist - mean;
			mean += delta/counter;
			M2 += delta * (dist - mean);
			
			Trace curdistor = new Trace(3);
			curdistor.time = cur.time;
			curdistor.values[0] = mean;
			curdistor.values[1] = dist;
			curdistor.values[2] = M2/counter;
			res.add(curdistor);
		}
		return res;
	}
}
