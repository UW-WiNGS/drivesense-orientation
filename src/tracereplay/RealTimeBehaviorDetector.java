package tracereplay;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import utility.Constants;
import utility.Log;
import utility.Trace;

public class RealTimeBehaviorDetector {

	public RealTimeBehaviorDetector() {
		
	}
	
	public List<Trace> projected_accelerometer = new ArrayList<Trace>();
	

	private List<Trace> window_accelerometer = new LinkedList<Trace>();
	private List<Trace> window_rotation_matrix = new LinkedList<Trace>();

	private RotationMatrix rm = new RotationMatrix();
	
	
	private Trace curSmoothedAccelerometer = null;
	private Trace curSmoothedGyroscope = null;

	final int kWindowSize = 10;

	
	
	public void processTrace(Trace trace) {
		String type = trace.type;
		if(type.equals(Trace.ACCELEROMETER)) {
			onAccelerometerChanged(trace);
		} else if (type.equals(Trace.GYROSCOPE)) {
			onGyroscopeChanged(trace);
		} else if(type.equals(Trace.ROTATION_MATRIX)) {
			window_rotation_matrix.add(trace);
			if(window_rotation_matrix.size() > kWindowSize) {
				window_rotation_matrix.remove(0);
			}
		} else {
			Log.log("Uncaptured trace type", trace.toString());
		}
	}
	
	public boolean stopped(List<Trace> window) {
		double [] maxs = {-100.0, -100.0, -100.0};
		double [] mins = {100.0, 100.0, 100.0};		
		final int dim = 3;
		final double threshold = 0.15;
		for(int i = 0; i < window.size(); ++i) {
			Trace cur = window.get(i);
			for(int j = 0; j < dim; ++j) {
				if(cur.values[j] > maxs[j]) {
					maxs[j] = cur.values[j];
				}
				if(cur.values[j] < mins[j]) {
					mins[j] = cur.values[j];
				}
			}
		}
		for(int i = 0; i < dim; ++i) {
			if(Math.abs(maxs[i] - mins[i]) > threshold) {
				return false;
			}
		}
		return true;
	}
	
	
	private Trace lowpassFilter(Trace last, Trace cur) {
		final double alpha = Constants.kExponentialMovingAverageAlpha;
		if(last == null) {
			last = new Trace(3);
			last.copyTrace(cur);
		} else {
			for(int j = 0; j < cur.dim; ++j) {
				cur.values[j] = alpha * cur.values[j] + (1.0 - alpha) * last.values[j];
			}
		}
		return cur;
	}
	
	
	private void onAccelerometerChanged(Trace accelerometer) {
		curSmoothedAccelerometer = lowpassFilter(curSmoothedAccelerometer, accelerometer);
		window_accelerometer.add(curSmoothedAccelerometer);
		if(window_accelerometer.size() > kWindowSize) {
			boolean steady = stopped(window_accelerometer);
			if(steady && rm.rm_set == false) {
				List<Trace> sub = PreProcess.extractSubList(window_rotation_matrix, window_accelerometer.get(0).time, window_accelerometer.get(kWindowSize - 1).time);
				Trace tmprm = PreProcess.getAverage(sub);
				if(tmprm !=null) {
					rm.rotation_matrix = tmprm.values;
					rm.setRotationMatrix(tmprm);
					rm.rm_set = true;
					Log.log("rotation matrix is set");
				}
			}
			window_accelerometer.remove(0);
		}
	}
	
	private void onGyroscopeChanged(Trace gyroscope) {
		curSmoothedGyroscope = lowpassFilter(curSmoothedGyroscope, gyroscope);
	}
}
