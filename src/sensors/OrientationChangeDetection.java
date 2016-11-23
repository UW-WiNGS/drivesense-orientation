package sensors;

import gpsevaluation.GPSAbstraction;
import gpsevaluation.TurnExtraction;
import io.DirectoryWalker;
import io.ReadWriteTrace;
import io.SqliteAccess;

import java.io.BufferedReader;
import java.util.ArrayList;
import java.util.List;

import utility.Constants;
import utility.Formulas;
import utility.Log;
import utility.Pattern;
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

	private static void processControlledTrips() {
		String outfolder = Constants.outputPath.concat("orientation/fixed/");
		List<String> folders = DirectoryWalker.getFolders(Constants.datPath);
		List<Trace> output = new ArrayList<Trace>();
		String type = "urban";
		int counter = 0;
		for(String directory: folders) {
			String folder = directory.concat("/" + type);
			List<Trip> trips = ReadWriteTrace.loadTrips(folder);
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
					tr.values[i - 1] = tmp.values[0];		
				}
				output.add(tr);
				
				
				/*
				if(counter >= 10) {
					return;
				}
				counter ++;
				List<Trace> distortionchange = trackClusterDistortion(trip.accelerometer_);
				ReadWriteTrace.writeFile(distortionchange, outfolder.concat("distortion" + String.valueOf(counter) + ".dat"));
				*/
			}
		}	
		ReadWriteTrace.writeFile(output, outfolder.concat("fixed_distortion.dat"));		
	}
	
	private static void processWildTrips() {
		List<String> files = DirectoryWalker.getFilePaths(Constants.kUncontrol.concat("holding"));
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

			Trace tr = new Trace(9);
			tr.time = trip.time_;
			for(int i = 1; i < 10; ++i) {
				tr.values[i - 1] = PreProcess.getTraceAt(distortionchange, i * 20 * 1000).values[0];		
			}
			output.add(tr);

			/*
			if(counter >= 10) {
				return;
			}
			ReadWriteTrace.writeFile(distortionchange, outfolder.concat("distortion" + String.valueOf(counter) + ".dat"));
			*/
		}
	
		//incrementalClustering(accelerometer);
	
		ReadWriteTrace.writeFile(output, outfolder.concat("holding_distortion.dat"));		
		
	}
	
	private static List<Trace> processTrip(Trip trip) {
		
		List<Trace> accelerometer = PreProcess.exponentialMovingAverage(trip.accelerometer_, -1);
		List<Trace> gyroscope = PreProcess.exponentialMovingAverage(trip.gyroscope_, -1);
		List<Trace> gps = PreProcess.exponentialMovingAverage(processGPS(trip.gps_), 3);
		
		
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
	private static List<Trace> trackClusterDistortion(List<Trace> cluster) {
		List<Trace> distortions = new ArrayList<Trace>();
		Trace center = new Trace(3);
		center.copyTrace(cluster.get(start_));
		int counter = 1;
		double distortion = 0.0;
		for(int i = start_ + 1; i < cluster.size(); ++i) {
			Trace cur = cluster.get(i);
			double dist = Formulas.euclideanDistance(center, cur);
			counter ++;
			for(int j = 0; j < center.dim; ++j) {
				center.values[j] += (1.0/counter)*(cur.values[j] - center.values[j]); 
			}
			distortion += (1.0/counter)*(dist - distortion);
			Trace curdistor = new Trace(1);
			curdistor.time = cur.time;
			curdistor.values[0] = distortion;
			distortions.add(curdistor);
		}
		return distortions;
	}
	
	private static double clusterDistortion(List<Trace> cluster) {
		double distortion = 0.0;
		Trace center = new Trace(3);
		center.copyTrace(cluster.get(start_));
		int counter = 1;
		for(int i = start_ + 1; i < cluster.size(); ++i) {
			Trace cur = cluster.get(i);
			double dist = Formulas.euclideanDistance(center, cur);
			counter ++;
			for(int j = 0; j < center.dim; ++j) {
				center.values[j] += (1.0/counter)*(cur.values[j] - center.values[j]); 
			}
			distortion += (1.0/counter)*(dist - distortion); 
		}
		return distortion;
	}
	
	public static void detectOrientationChange() {
		
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
	
	private static List<Trace> processGPS(List<Trace> gps) {
		List<Trace> res = new ArrayList<Trace>();
		for(int i = 0; i < gps.size() - 1; ++i) {
			Trace cur = gps.get(i);
			Trace ntr = new Trace(6);
			ntr.time = cur.time;
			ntr.type = cur.type;
			for(int j = 0; j < cur.dim; ++j) {
				ntr.values[j] = cur.values[j];
			}			
			ntr.values[3] = (gps.get(i + 1).values[2] - cur.values[2])/((gps.get(i + 1).time - cur.time)/1000.0);
			for(int j = i + 1; j < gps.size(); ++j) {
				Trace next = gps.get(j);
				double dist = GPSAbstraction.distance(cur, next);
				if(dist <= 10.0) continue;
				double direction = GPSAbstraction.direction(cur, next);
				ntr.values[4] = direction;
				break;			
			}
			int sz = res.size();
			if(sz >= 1) {
				Trace pre = res.get(sz - 1);
				pre.values[5] = GPSAbstraction.turnAngle(pre.values[4], ntr.values[4])/((ntr.time - pre.time)/1000.0);
					
			}
			res.add(ntr);
		}
		return res;
		
	}
 
}
