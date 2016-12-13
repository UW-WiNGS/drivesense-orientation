package tracereplay;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import sensors.SensorCluster;
import utility.Constants;
import utility.Formulas;
import utility.Log;
import utility.PreProcess;
import utility.Trace;

public class RealTimeSensorProcessing {
	
	private static final String TAG = "RealTimeSensorProcessing";

	public RealTimeSensorProcessing() {
		
	}
	
	private List<Trace> window_accelerometer = new LinkedList<Trace>();
	private List<Trace> window_gyroscope = new LinkedList<Trace>();
	private List<Trace> window_rotation_matrix = new LinkedList<Trace>();
	private Trace curSmoothedAccelerometer = null;
	private Trace curSmoothedGyroscope = null;
	final int kWindowSize = 15;
	
	
		
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
		} else if(type.equals(Trace.GPS)) {
			onGPSChanged(trace);
		} else if(type.equals(Trace.MAGNETOMETER)){
			onMagnetometerChanged(trace);
		} else {
			Log.d("Uncaptured trace type", trace.toString());
		}
	}
	
	public List<List<Trace>> getTrainSet() {
		return trainset_;
	}
	public Trace getInitRM() {
		return initRM_;
	}
	public Trace getHorizontalRM() {
		return this.horizontalRM_;
	}
	
	public Trace getGyroDrift() {
		return this.gyrodrift_;
	}
	
	private boolean stopped_ = false;
	private List<List<Trace>> trainset_ = new ArrayList<List<Trace>>();
	private List<Trace> trainsample_ = null;
	
	private Trace initRM_ = null;
	private Trace horizontalRM_ = null;
	private Trace verticalRM_ = null;
	
	private double gravity_ = 0.0;
	
	private int train_len_ = 30;
	public void setTrainLength(int len) {
		this.train_len_ = len;
	}
	public int getTrainLength() {
		return this.train_len_;
	}
	
	private void onMagnetometerChanged(Trace magnetometer) {
		
	}
	
	private void onGPSChanged(Trace gps) {
		
	}
	
	private void onAccelerometerChanged(Trace accelerometer) {
		
		curSmoothedAccelerometer = lowpassFilter(curSmoothedAccelerometer, accelerometer);
		window_accelerometer.add(curSmoothedAccelerometer);
		
		detectOrientationChange(curSmoothedAccelerometer);
		monitorStability(curSmoothedAccelerometer);
		
		if(window_accelerometer.size() >= kWindowSize) {
			stopped_ = stopped(window_accelerometer);
			
			
			if(straight_) {
				//train the initial rotation matrix
				//opportunity to train the oritentation matrix
				if(gravity_ <= 0.0 &&  stopped_ == true) {
					gravity_ = 0.0;
					Trace tmp = PreProcess.getAverage(window_accelerometer);
					for(int j = 0; j < tmp.dim; ++j) {
						gravity_ += tmp.values[j] * tmp.values[j];
					}
					gravity_ = Math.sqrt(gravity_);
					initRM_ = PreProcess.getAverage(window_rotation_matrix);
				}
					
				if(null == trainsample_) {
					trainsample_ = new ArrayList<Trace>();
					for(int i = 0; i < window_accelerometer.size(); ++i) {
						trainsample_.add(window_accelerometer.get(i));
					}
				} else {
					trainsample_.add(curSmoothedAccelerometer);
				}
			} else {
				//put current train sample into train set
				if(trainsample_ != null) {
					int trainlen = trainsample_.size();
					
					//we use small segment to train
					if(trainlen >= this.train_len_) {
						trainset_.add(trainsample_);		
					}
					
					if(trainlen >= this.train_len_) {
						//training opportunity
						calculateHorizontalRM(trainsample_);
					}
					if (trainlen >= this.train_len_) {
						calibrateByAccelerometer(trainsample_);
					}
					
				}
				trainsample_ = null;
			}
			window_accelerometer.remove(0);
		}
	}
	
	public Trace calculateDirectionVector(List<Trace> trainsample) {
		Log.d(TAG, "calculateDirectionVector");
		Trace direction = new Trace(3);
		List<Trace> window = new ArrayList<Trace>();
		List<Integer> pattern = new ArrayList<Integer>();		
		for(Trace trace: trainsample) {
			window.add(trace);
			if(window.size() >= kWindowSize) {
				boolean issteady = stopped(window);
				int label = issteady == true ? 1: -1;
				int sz = pattern.size();
				if(sz == 0) {
					pattern.add(label);
				} else {
					if(pattern.get(sz - 1)/label > 0) {
						Integer cur = pattern.get(sz - 1).intValue() + label;
						pattern.remove(sz - 1);
						pattern.add(cur);
					} else {
						pattern.add(label);
					}
				}
				window.remove(0);
			}
		}
		int pos = 0;
		for(Integer cur: pattern) {
			if(cur.intValue() < 0) {
				Trace sub = PreProcess.getAverage(trainsample.subList(pos, pos + Math.abs(cur.intValue())));
			}
			pos += Math.abs(cur);
		}
		return direction;
	}


	
	private void calibrateByAccelerometer(List<Trace> trainsample) {
		if(this.initRM_ == null || this.horizontalRM_ == null) {
			return;
		}
		List<Trace> aligned = new ArrayList<Trace>();
		for(Trace trace: trainsample) {
			Trace tmp = Formulas.rotate(trace, this.initRM_.values);
			Trace cur = Formulas.rotate(tmp, this.horizontalRM_.values);
			aligned.add(cur);
		}
		Trace avg = PreProcess.getAverage(aligned);
	}
	
	private double coeff1 = 0.0;
	private int coeffcounter = 0;
	private void calculateHorizontalRM(List<Trace> trainsample) {
		if(initRM_ == null /*|| this.horizontalRM_ != null*/) {
			return;
		}
		List<Trace> sample = new ArrayList<Trace>();
		for(Trace trace: trainsample) {
			Trace cur = Formulas.rotate(trace, initRM_.values);
			sample.add(cur);
		}
		double [] coeff = Formulas.curveFit(sample, 0, 1);
	
		coeff1 = (sample.size() * coeff[1] + coeff1 * coeffcounter) / (sample.size() + coeffcounter);
		coeffcounter += sample.size();
		
		
		Trace hdiff = new Trace(3);
		hdiff.setValues(1.0, coeff1, 0.0);
		Trace unit = Formulas.getUnitVector(hdiff);
		Trace yaxe = new Trace(3);
		yaxe.setValues(0.0, 1.0, 0.0);
		this.horizontalRM_ = Formulas.rotationMatrixBetweenHorizontalVectors(unit, yaxe);
	}
	
	
	private boolean straight_ = false;
	private Trace gyrodrift_ = new Trace(3);
	private void onGyroscopeChanged(Trace gyroscope) {
		curSmoothedGyroscope = lowpassFilter(curSmoothedGyroscope, gyroscope);
		window_gyroscope.add(curSmoothedGyroscope);
		if(window_gyroscope.size() >= kWindowSize) {
			straight_ = isStraight(window_gyroscope);
			window_gyroscope.remove(0);
		}
	}
	
	/**
	 * check if the car is moving straight by gyroscope
	 * @param window
	 * @return
	 */
	public boolean isStraight(List<Trace> window) {
		//final double threshold = 0.005; 
		final double threshold = 0.004; //0.008;
		
		double[] devi = Formulas.standardDeviation(window);
		for(int i = 0; i < devi.length; ++i) {
			if(devi[i] >= threshold) {
				return false;
			}
		}
		return true;
	}
	
	/**
	 * check if the car is stopped by accelerometer
	 * @param window
	 * @return
	 */
	public boolean stopped(List<Trace> window) {
		final double threshold = 0.004;
		double variance = SensorCluster.calculateClusterVariance(window);
		if(variance <= threshold) {
			return true;
		} else {
			return false;
		}
		
	}
	
	
	public Trace lowpassFilter(Trace last, Trace cur) {
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
	
	private List<Trace> orientation_buffer_ = new ArrayList<Trace>();
	private boolean orientation_changing_ = false;
	private int number_of_orientation_changed = 0;
	
	public void detectOrientationChange(Trace accelerometer) {
		
		orientation_buffer_.add(accelerometer);
		if(orientation_buffer_.size() > 30) orientation_buffer_.remove(0);						
		
		double m2 = calculateClusterVariance(orientation_buffer_);
		
		if(m2 > Constants.kOrientationChangeVarianceThreshold) {
			if(orientation_changing_ == false) {
				//start changeing
				number_of_orientation_changed++;
			}
			orientation_changing_ = true;
		} else {
			if(orientation_changing_ == true) {
				orientation_changing_ = false;
				//chaning ends
			}
		}
	}
	
	private List<Trace> mv_buffer_ = new ArrayList<Trace>();
	private double avg_mv_ = 0.0;
	private int mv_counter_ = 0;
	public void monitorStability(Trace accelerometer) {
		
		if(this.orientation_changing_ == true) {
			mv_buffer_.clear();
		}
		
		mv_buffer_.add(accelerometer);
		if(mv_buffer_.size() > 600) mv_buffer_.remove(0);						
		
		double m2 = calculateClusterVariance(mv_buffer_);
		double sum = avg_mv_ * mv_counter_ + m2;
		avg_mv_ = sum / (++mv_counter_);
	}
	
}
