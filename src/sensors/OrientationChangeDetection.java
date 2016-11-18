package sensors;

import gpsevaluation.GPSAbstraction;
import io.DirectoryWalker;
import io.ReadWriteTrace;
import io.SqliteAccess;

import java.io.BufferedReader;
import java.util.ArrayList;
import java.util.List;

import utility.Constants;
import utility.Log;
import utility.Trace;
import utility.Trip;

public class OrientationChangeDetection {
	
	private static final String TAG = "OrientationChangeDetection";
	public static void start() {
		List<String> files = DirectoryWalker.getFilePaths(Constants.kUncontrol.concat("suman"));
		double sum = 0.0;
		Log.log(TAG, files.size());
		List<Trace> accelerometer = new ArrayList<Trace>();
		for(String file: files) {
			String name = file.substring(file.length() - 16, file.length() - 3);
			long time = Long.valueOf(name);
			Trip trip = SqliteAccess.loadTrip(file, time);
			double dist = GPSAbstraction.accumulatedDistance(trip.gps_);
			sum += dist;
			Log.log(TAG, file, dist);
			for(int i = 0; i < trip.accelerometer_.size(); ++i) {
				if(i % 20 == 0) {
					Trace cur = trip.accelerometer_.get(i);
					accelerometer.add(cur);
				}
			}
			break;
		}
		Log.log(TAG, sum);
	
		incrementalClustering(accelerometer);
		ReadWriteTrace.writeFile(accelerometer, Constants.outputPath.concat("orientation/data/accelerometer.dat"));
		
	}
		
	public static void incrementalClustering(List<Trace> accelerometer) {
		
	}
 
}
