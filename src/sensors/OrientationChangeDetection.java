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
		
		//processControlledTrips();		
		//processWildTrips();
	}
	
	public static void detectOrientationChange() {
		String output = Constants.outputPath + "orientation/orientationchange/";
		String file = Constants.kUncontrol + "controlled/onechange/1480430491380.db";
		String name = file.substring(file.length() - 16, file.length() - 3);
		long time = Long.valueOf(name);
		Trip trip = SqliteAccess.loadTrip(file, time);

		List<Trace> accelerometer = processTrip(trip);			
		double distortion = clusterDistortion(accelerometer);
		Log.log(TAG, distortion);
		
		Trace center = new Trace(3);
		
		int start = 400;

		SensorCluster cluster = new SensorCluster();
		cluster.initCenter(accelerometer.get(start));
		SensorCluster ncluster = new SensorCluster();
		
		Trace tracking = new Trace(3);
		List<Trace> buffer = new LinkedList<Trace>();
		
		int counter = 0;
		double mean = 0.0, M2 = 0.0;
		
		List<Trace> res = new ArrayList<Trace>();
		for(int i = start_ + 1; i < accelerometer.size(); ++i) {
			Trace cur = accelerometer.get(i);
			tracking.copyTrace(cur);
			double tocluster = cluster.distanceToCenter(cur);
			double tolast = Formulas.euclideanDistance(cur, tracking);
			
			if(Math.abs(tocluster) > 3.0) {
				Log.log(TAG, cur.time, tocluster);
			} else {				
				cluster.addAccelerometer(cur);
			}
			buffer.add(cur);
			if(buffer.size() >= 50) buffer.remove(0);
			double m2 = clusterDistortion(buffer);
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
				double distortion = clusterDistortion(accelerometer);
				//Log.log(TAG, distortion);
				Trace tr = new Trace(9);
				tr.time = trip.time_;
				
				List<Trace> distortionchange = trackClusterDistortion(accelerometer);
				
				
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
				
				/*
				if(counter >= 10) {
					break;
				}
				counter ++;
				ReadWriteTrace.writeFile(distortionchange, outfolder.concat("distortion" + String.valueOf(counter) + ".dat"));
				*/				
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
			double distortion = clusterDistortion(accelerometer);
			Log.log(TAG, distortion);
			
			List<Trace> distortionchange = trackClusterDistortion(accelerometer);
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
	
	private static final int start_ = 0; // seconds after start
	public static List<Trace> trackClusterDistortion(List<Trace> cluster) {
		List<Trace> res = new ArrayList<Trace>();
		Trace center = new Trace(3);
		center.copyTrace(cluster.get(0));
		int counter = 0;
		double mean = 0.0, M2 = 0.0;
		for(int i = start_ + 1; i < cluster.size(); ++i) {
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
	
	public static double clusterDistortion(List<Trace> cluster) {
		double mean = 0.0, M2 = 0.0;
		Trace center = new Trace(3);
		center.copyTrace(cluster.get(0));
		int counter = 0;
		for(int i = start_ + 1; i < cluster.size(); ++i) {
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
	

	public static void incrementalClustering(List<Trace> accelerometer) {
		int k = 2;
		Trace [] center = new Trace[k];
		int [] counter = new int[k];
		for(int i = 0; i < k; ++i) {
			center[i] = new Trace(3);
			center[i].copyTrace(accelerometer.get(i));
			counter[i] = 1;
		}
		for(int i = k; i < accelerometer.size(); ++i) {
			//find the closest
			double maxdist = Double.MAX_VALUE;
			Trace cur = accelerometer.get(i);	
			int index = -1;
			for(int j = 0; j < k; ++j) {
				double dist = Formulas.euclideanDistance(center[j], cur);
				if(dist < maxdist) {
					maxdist = dist;
					index = j;
				}
			}
			counter[index]++;
			for(int j = 0; j < center[index].dim; ++j) {
				center[index].values[j] += (1.0/counter[index])*(cur.values[j] - center[index].values[j]); 
			}
		}
	}
	

 
}
