package sensors;

import gpsevaluation.GPSAbstraction;
import io.DirectoryWalker;
import io.ReadWriteTrace;
import io.SqliteAccess;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import utility.Constants;
import utility.Formulas;
import utility.Log;
import utility.PreProcess;
import utility.Trace;
import utility.Trip;

public class OrientationChangeDetection {
	
	private static final String TAG = "OrientationChangeDetection";
	public static void start() {
		
		detectOrientationChange();
		
		//flushSqliteData();
		
		
		//processControlledTrips();		
		//processWildTrips();
		
		//evaluateOrientaionChangeDetection();
	}
	
	private static void evaluateOrientaionChangeDetection() {
		String input = Constants.kUncontrol + "controlled/orientation_change/";
		String output = Constants.outputPath + "orientation/orientationchange/data/";
		List<String> files = DirectoryWalker.getFileNames(input);
		Log.log(TAG, files);
		for(String file: files) {
			String name = file.substring(file.length() - 16, file.length() - 3);
			long time = Long.valueOf(name);
			Trip trip = SqliteAccess.loadTrip(input.concat(file), time);
			List<Trace> res = orientationChangeDetection(trip);
			//ReadWriteTrace.writeFile(res, output.concat("m2.dat"));
		}
	}
	
	public static boolean orientationChanged(List<Trace> accelerometer) {
		SensorCluster cluster = new SensorCluster();
		cluster.initCenter(accelerometer.get(0));
		List<Trace> buffer = new LinkedList<Trace>();		
		List<Trace> res = new ArrayList<Trace>();

		boolean transition = false;
		for(int i = 1; i < accelerometer.size(); ++i) {
			Trace cur = accelerometer.get(i);
			
			buffer.add(cur);
			if(buffer.size() >= 30) buffer.remove(0);						
			double m2 = SensorCluster.calculateClusterVariance(buffer);
			
			if(m2 > Constants.kOrientationChangeVarianceThreshold) {
				if(transition == false) {
					//Log.log(TAG, cur.time);
				}
				transition = true;
				return true;
			} else {
				if(transition == true) {
					Log.log(TAG, cur.time, m2);
					transition = false;
					break;
				}
			}
			cluster.addAccelerometer(cur);			
		}
		return false;
		
	}
	
	private static List<Trace> orientationChangeDetection(Trip trip) {
	
		List<Trace> accelerometer = processTrip(trip);
		SensorCluster cluster = new SensorCluster();
		
		int start = 400;
		
		cluster.initCenter(accelerometer.get(start));
		List<Trace> buffer = new LinkedList<Trace>();		
		List<Trace> res = new ArrayList<Trace>();

		boolean transition = false;
		for(int i = start + 1; i < accelerometer.size(); ++i) {
			Trace cur = accelerometer.get(i);
			double tocluster = cluster.distanceToCenter(cur);
			
			buffer.add(cur);
			if(buffer.size() >= 30) buffer.remove(0);						
			double m2 = SensorCluster.calculateClusterVariance(buffer);
			
			if(m2 > Constants.kOrientationChangeVarianceThreshold) {
				if(transition == false) {
					Log.log(TAG, cur.time, trip.path);
				}
				transition = true;
			} else {
				if(transition == true) {
					Log.log(TAG, cur.time, m2);
					transition = false;
					break;
				}
			}
			cluster.addAccelerometer(cur);			
			Trace ntr = new Trace(1);
			ntr.time = cur.time;
			ntr.values[0] = m2;
			res.add(ntr);
		}
		return res;
	}
	
	public static void detectOrientationChange() {
		String output = Constants.outputPath + "orientation/orientationchange/";
		String file = Constants.kUncontrol + "controlled/onechange/1480430491380.db";
		String name = file.substring(file.length() - 16, file.length() - 3);
		long time = Long.valueOf(name);
		Trip trip = SqliteAccess.loadTrip(file, time);

		List<Trace> accelerometer = processTrip(trip);			
		int start = 0;

		SensorCluster cluster = new SensorCluster();
		cluster.initCenter(accelerometer.get(start));

		List<Trace> buffer = new LinkedList<Trace>();
		
		List<Trace> res = new ArrayList<Trace>();
		boolean transition = false;
		for(int i = start + 1; i < accelerometer.size(); ++i) {
			Trace cur = accelerometer.get(i);
			double tocluster = cluster.distanceToCenter(cur);
			
			buffer.add(cur);
			if(buffer.size() >= 30) buffer.remove(0);						
			double m2 = SensorCluster.calculateClusterVariance(buffer);
			
			if(m2 > Constants.kOrientationChangeVarianceThreshold) {
				if(transition == false) {
					Log.log(TAG, cur.time, "start");
				}
				transition = true;
			} else {
				if(transition == true) {
					Log.log(TAG, cur.time, m2);
					transition = false;
				}
			}
			cluster.addAccelerometer(cur);			
			Trace ntr = new Trace(1);
			ntr.time = cur.time;
			ntr.values[0] = m2;
			res.add(ntr);
		}
				
		ReadWriteTrace.writeFile(accelerometer, output.concat("pocket_vertical.dat"));
		ReadWriteTrace.writeFile(res, output.concat("m2.dat"));

	}
		
	

	private static void processControlledTrips() {
		String outfolder = Constants.outputPath.concat("orientation/fixed/");
		List<String> folders = DirectoryWalker.getFolders(Constants.datPath);
		List<Trace> output = new ArrayList<Trace>();
		
		int counter = 0;
		for(String directory: folders) {
			List<Trip> trips = ReadWriteTrace.loadTrips(directory.concat("/urban"));
			List<Trip> highway = ReadWriteTrace.loadTrips(directory.concat("/highway"));
			trips.addAll(highway);
			for(Trip trip: trips) {				
				List<Trace> accelerometer = processTrip(trip);
				double distortion = SensorCluster.calculateClusterVariance(accelerometer);
				//Log.log(TAG, distortion);
				Trace tr = new Trace(9);
				tr.time = trip.time_;
				
				List<Trace> distortionchange = SensorCluster.trackClusterVariance(accelerometer);
				
				
				long end = distortionchange.get(distortionchange.size() - 1).time;
				if(end <= 200 * 1000) continue;
				for(int i = 1; i < 10; ++i) {
					Trace tmp = PreProcess.getTraceAt(distortionchange, i * 20 * 1000);
					tr.values[i - 1] = tmp.values[2];		
				}
				boolean outlier = false;
				for(int i = 1; i < 10; ++i) {
					if(tr.values[i - 1] > 1.5) {
						outlier = true;
						break;
					}
				}
				if(outlier) continue;
				output.add(tr);
			}
		}	
		ReadWriteTrace.writeFile(output, outfolder.concat("fixed_distortion.dat"));		
	}
	
	private static void processWildTrips() {
		List<String> files = DirectoryWalker.getFilePaths(Constants.kUncontrol.concat("controlled/holding"));
		String outfolder = Constants.outputPath.concat("orientation/holding/");
		List<Trace> output = new ArrayList<Trace>();
		double sum = 0.0;
		int counter = 0;
		for(String file: files) {
			counter++;
			String name = file.substring(file.length() - 16, file.length() - 3);
			long time = Long.valueOf(name);
			Trip trip = SqliteAccess.loadTrip(file, time);
			double dist = GPSAbstraction.accumulatedDistance(trip.gps_);
			sum += dist;

			Log.log(file, dist, trip.path);			
			List<Trace> accelerometer = processTrip(trip);			
			double distortion = SensorCluster.calculateClusterVariance(accelerometer);
			Log.log(TAG, distortion);
			
			List<Trace> distortionchange = SensorCluster.trackClusterVariance(accelerometer);
			long end = distortionchange.get(distortionchange.size() - 1).time;
			if(end <= 100 * 1000) continue;

			/*
			Trace tr = new Trace(9);
			tr.time = trip.time_;
			for(int i = 1; i < 10; ++i) {
				tr.values[i - 1] = PreProcess.getTraceAt(distortionchange, i * 20 * 1000).values[0];		
			}
			output.add(tr);
			*/
			
			if(counter >= 10) {
				return;
			}
			ReadWriteTrace.writeFile(distortionchange, outfolder.concat("distortion" + String.valueOf(counter) + ".dat"));
			
		}
	
		//incrementalClustering(accelerometer);
	
		//ReadWriteTrace.writeFile(output, outfolder.concat("holding_distortion.dat"));		
		
	}
	
	private static List<Trace> processTrip(Trip trip) {
		
		List<Trace> accelerometer = PreProcess.exponentialMovingAverage(trip.accelerometer_, -1);
		List<Trace> gyroscope = PreProcess.exponentialMovingAverage(trip.gyroscope_, -1);
		List<Trace> gps = PreProcess.exponentialMovingAverage(GPSAbstraction.wrapperGPS(trip.gps_), 3);
		
		
		List<Trace> res = new ArrayList<Trace>();
		
		/*
		for(int i = 0; i < gps.size(); ++i) {
			Trace cur = gps.get(i);
			if(Math.abs(cur.values[3]) < 0.5) {
				Trace curacc = PreProcess.getTraceAt(accelerometer, cur.time);
				res.add(curacc);
			}
			
		}
		*/
		
		int sample_rate = 1;
		for(int i = 0; i < accelerometer.size(); ++i) {
			Trace cur = accelerometer.get(i);
			if(i % sample_rate == 0) {
				res.add(cur);
			}			
		}
		return res;
		/*
		ReadWriteTrace.writeFile(accelerometer, Constants.outputPath.concat("orientation/datasync/accelerometer.dat"));
		ReadWriteTrace.writeFile(gyroscope, Constants.outputPath.concat("orientation/datasync/gyroscope.dat"));
		ReadWriteTrace.writeFile(gps, Constants.outputPath.concat("orientation/datasync/gps.dat"));
		*/
	}
	

	

 
}
