package tracereplay;

import java.util.LinkedList;
import java.util.List;

import utility.Constants;
import utility.Log;
import utility.Trace;

public class RealTimeTiltCalculation {
	private static final String TAG = "RealTimeTiltCalculation";

	public RealTimeTiltCalculation() {
		
	}
	
	private List<Trace> window_accelerometer = new LinkedList<Trace>();
	private List<Trace> window_gyroscope = new LinkedList<Trace>();
	private List<Trace> window_rotation_matrix = new LinkedList<Trace>();
	private Trace curSmoothedAccelerometer = null;
	private Trace curSmoothedGyroscope = null;
	final int kWindowSize = 10;
		
	private double curTilt = 0.0;
	
	public double getTilt() {
		return this.curTilt;
	}
	/**
	 * the only input point
	 * @param trace
	 */
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
			Log.d("Uncaptured trace type", trace.toString());
		}
	}
	
	
	private void onGyroscopeChanged(Trace gyroscope) {
		curSmoothedGyroscope = lowpassFilter(curSmoothedGyroscope, gyroscope);
		window_gyroscope.add(curSmoothedGyroscope);
		if(window_gyroscope.size() >= kWindowSize) {
			window_gyroscope.remove(0);
		}
		
	}
	
	
	private void onAccelerometerChanged(Trace accelerometer) {		
		curSmoothedAccelerometer = lowpassFilter(curSmoothedAccelerometer, accelerometer);
		window_accelerometer.add(curSmoothedAccelerometer);
		if(window_accelerometer.size() >= kWindowSize) {
			window_accelerometer.remove(0);
		}
		double x = curSmoothedAccelerometer.values[0];
		double z = curSmoothedAccelerometer.values[2];
		double angle = 0.0;
		if(z == 0.0) {
			angle = x > 0.0 ? -1.57 : 1.57;
		} else {
			angle = Math.atan(-x/z);
		}
		this.curTilt = Math.toDegrees(angle);
	}
	
	private Trace lowpassFilter(Trace last, Trace cur) {
		final double alpha = Constants.kExponentialMovingAverageAlpha;
		Trace res = new Trace(cur.dim);
		res.copyTrace(cur);
		if(last != null) {
			for(int j = 0; j < cur.dim; ++j) {
				res.values[j] = alpha * cur.values[j] + (1.0 - alpha) * last.values[j];
			}
		}
		return res;
	}
}
