package utility;

import io.ReadWriteTrace;

import java.util.ArrayList;
import java.util.List;


public class Trip {
	public static boolean nosensor_ = false;
	private static String TAG = "Trip";
	public long time_;
	public String name_;
	public static double minAcc = 0.0;
	/**
	 * statistics
	 */
	public static double kHighwaySpeed_ = 50; //in mph
	public long duration_ = 0; //in millisecond
	public double distance_ = 0.0; //in mile
	public double highway_distance_ = 0.0; //in mile
	
	
	/**
	 * OBD data
	 */
	public List<Trace> fl_; //fuel level
	public List<Trace> maf_; //massive air flow
	public List<Trace> ltft_;
	public List<Trace> stft_;
	public List<Trace> fss_;
	public List<Trace> speed_;
	public List<Trace> acc_;
	public List<Trace> rpm_;
	
	/**
	 * Sensor data
	 */
	public List<Trace> accelerometer_;
	public List<Trace> rotation_matrix_;
	public List<Trace> gyroscope_;
	public List<Trace> gps_;
	public List<Trace> gps_elevation_;
	
	public String path;

	public Trip() {
		fl_ = new ArrayList<Trace>();
		maf_ = new ArrayList<Trace>();
		ltft_ = new ArrayList<Trace>();
		stft_ = new ArrayList<Trace>();
		fss_ = new ArrayList<Trace>();
		speed_ = new ArrayList<Trace>();
		acc_ = new ArrayList<Trace>();
		rpm_ = new ArrayList<Trace>();
		
		if(false == nosensor_) {
			accelerometer_ = new ArrayList<Trace>();
			gyroscope_ = new ArrayList<Trace>();
			rotation_matrix_ = new ArrayList<Trace>();
		}
		gps_ = new ArrayList<Trace>();
		gps_elevation_ = new ArrayList<Trace>();
	}
	
	public void readOBDDataFromFolder(String folder) throws Exception {
		
		path = folder;
		speed_ = ReadWriteTrace.readFile(folder.concat("speed.dat"), 1);
		/*
		maf_ = ReadWriteTrace.readFile(folder.concat("maf.dat"), 1);
		fl_ = ReadWriteTrace.readFile(folder.concat("fuellevel.dat"), 1);
		ltft_ = ReadWriteTrace.readFile(folder.concat("ltft1.dat"), 1);
		stft_ = ReadWriteTrace.readFile(folder.concat("stft1.dat"), 1);
		fss_ = ReadWriteTrace.readFile(folder.concat("fss.dat"), 1);
		rpm_ = ReadWriteTrace.readFile(folder.concat("rpm.dat"), 1);
		
		acc_ = ReadWriteTrace.readFile(folder.concat("acc.dat"), 1);
		processAcc();
		*/
	}
	
	/**
	 * process the accelerator value, make it starts from 0
	 */
	private void processAcc() {
		double mini = 100;
		for(Trace trace: acc_) {
			if(trace.values[0] < mini)
				mini = trace.values[0];
		}
		if(mini==0.0) return;
		minAcc = mini;
		for(Trace trace: acc_) {
			trace.values[0] -= mini;
		}
	}
	
	/**
	 * we comment out all sensor data except 
	 * @param folder
	 */
	public void readSensorDataFromFolder(String folder) throws Exception {
		gps_ = ReadWriteTrace.readFile(folder.concat("gps.dat"), 2);
		gps_elevation_ = ReadWriteTrace.readFile(folder.concat("gps_sychronized.dat"), 4);
		if(false == Trip.nosensor_) {
			accelerometer_ = ReadWriteTrace.readFile(folder.concat("accelerometer.dat"), 3);
			gyroscope_ = ReadWriteTrace.readFile(folder.concat("gyroscope.dat"), 3);
			rotation_matrix_ = ReadWriteTrace.readFile(folder.concat("rotation_matrix.dat"), 9);
		}
	}
	
	public void flushOBDDataToFolder(String outfolder) {
		Log.log(TAG, "flush obd data to " + outfolder);
		ReadWriteTrace.writeFile(fl_, outfolder.concat("fuellevel.dat"));
		ReadWriteTrace.writeFile(speed_, outfolder.concat("speed.dat"));
		ReadWriteTrace.writeFile(rpm_, outfolder.concat("rpm.dat"));
		ReadWriteTrace.writeFile(maf_, outfolder.concat("maf.dat"));
		ReadWriteTrace.writeFile(acc_, outfolder.concat("acc.dat"));
		ReadWriteTrace.writeFile(fss_, outfolder.concat("fss.dat"));
		ReadWriteTrace.writeFile(ltft_, outfolder.concat("ltft1.dat"));
		ReadWriteTrace.writeFile(stft_, outfolder.concat("stft1.dat"));
	}
	
	public void flushSensorDataToFolder(String outfolder) {
		Log.log(TAG, "flush sensor data to " + outfolder);
		ReadWriteTrace.writeFile(gps_, outfolder.concat("gps.dat"));
		if(false == Trip.nosensor_) {
			Log.error(accelerometer_.size(), outfolder.concat("accelerometer.dat"));
			ReadWriteTrace.writeFile(accelerometer_, outfolder.concat("accelerometer.dat"));
			ReadWriteTrace.writeFile(gyroscope_, outfolder.concat("gyroscope.dat"));
			ReadWriteTrace.writeFile(rotation_matrix_, outfolder.concat("rotation_matrix.dat"));
		}
	}
	
	
	public void setTime(long time) {
		time_ = time;
	}
	
	
	/**
	 * @return the distance traveled from this trip in miles
	 */	
	public void tripStatistics() {
		int sz = speed_.size();
		if(sz < 1) return;
		duration_ = speed_.get(sz - 1).time - speed_.get(0).time;
		for(int i = 0; i < speed_.size() - 1; ++i) {
			Trace trace = speed_.get(i);
			double diff = speed_.get(i + 1).time - trace.time;
			diff /= 3600000.0; //time difference in hour
			distance_ += trace.values[0] * Constants.kKmPHToMPH * diff; //km/h
			if(trace.values[0] * Constants.kKmPHToMPH > kHighwaySpeed_) {
				highway_distance_ += trace.values[0] * Constants.kKmPHToMPH * diff;
			}
		}
	}
	
	public long duration() {
		return duration_;
	}
	
	public double tripDistance() {
		return distance_;
	}
	public double highwayDistance() {
		return highway_distance_;
	}
	
	public boolean isHighwayTrip() {
		if(highway_distance_ > 1.0) {
			return true;
		} else {
			return false;
		}
	}
	
	public List<Trace> union() {
		List<Trace> all = new ArrayList<Trace>();
		double rate = 1.0;
		//by bozhao
		if(this.speed_.size() == 0)
			return all;
		List<Trace> speed = PreProcess.interpolate(this.speed_, rate);
		List<Trace> rpm = PreProcess.interpolate(this.rpm_, rate);
		List<Trace> acc = PreProcess.interpolate(this.acc_, rate);
		List<Trace> maf = PreProcess.interpolate(this.maf_, rate);
		List<Trace> stft = PreProcess.interpolate(this.stft_, rate);
		List<Trace> ltft = PreProcess.interpolate(this.ltft_, rate);

		for(int i = 0; i < speed.size(); ++i) {
			Trace pspeed = speed.get(i);
			long time = pspeed.time;
			Trace prpm = PreProcess.getTraceAt(rpm, time);
			Trace pacc = PreProcess.getTraceAt(acc, time);
			Trace pmaf = PreProcess.getTraceAt(maf, time);
			Trace pltft = PreProcess.getTraceAt(ltft, time);
			Trace pstft = PreProcess.getTraceAt(stft, time);
			if(null == pmaf || null == pacc || null == prpm)
				continue;
			double srate = 1.0, lrate = 1.0;
			if(null != pstft && null!= pltft) {
				srate = (1 + pstft.values[0]/100.0);
				lrate = (1 + pltft.values[0]/100.0);
			}			
			Trace ntr = new Trace(4);
			ntr.time = time;
			ntr.values[0] = pspeed.values[0]; //speed
			ntr.values[1] = prpm.values[0]; //rpm
			ntr.values[2] = pacc.values[0]; // acc
			ntr.values[3] = pmaf.values[0] * srate * lrate; //maf
			all.add(ntr);
		}
		return all;
	}
	
	
	
	/**
	 * @param maf: sample rate must be 1 sample per second
	 * @return
	 */
	public static double fuelConsumptionByMAF(List<Trace> maf) {	
		double res = 0.0;
		int sz = maf.size();
		for(int i = 0; i < sz - 1; ++i) {
			long time = maf.get(i + 1).time - maf.get(i).time;
			if(time < 0) {
				Log.log(TAG, "different trip");
				continue;
			}
			double sec = time / 1000.0;
			double value = maf.get(i).values[0];
			double gallon = (value * sec)/(Constants.kAirFuelRatio * 6.17 * 454);
			res += gallon;
		}
		return res;
	}
	
}
