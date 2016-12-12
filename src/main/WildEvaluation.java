package main;

import io.DirectoryWalker;
import io.ReadWriteTrace;
import io.SqliteAccess;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import sensors.SensorCluster;
import utility.Constants;
import utility.Log;
import utility.PreProcess;
import utility.Trace;
import utility.Trip;

public class WildEvaluation {
	
	private static final String TAG = "WildEvaluation";
	
	public static void start() {
		wildOrientationChanges();
	}
	
	
	private static void wildOrientationChanges() {
		String input = Constants.kUncontrol.concat("suman/");
		//String input = Constants.kUncontrol.concat("controlled/holding/");
		
		String output = Constants.outputPath.concat("orientation/varies/");
		
		
		List<String> suman = DirectoryWalker.getFilePaths(Constants.kUncontrol.concat("suman/"));
		List<String> lei = DirectoryWalker.getFilePaths(Constants.kUncontrol.concat("lei/"));
		List<String> others = DirectoryWalker.getFilePaths(Constants.kUncontrol.concat("others/"));
		
		List<String> files = suman;
		files.addAll(lei);
		files.addAll(others);
		
		List<Trace> res = new ArrayList<Trace>();
		for(String file: files) {
			String name = file.substring(file.length() - 16, file.length() - 3);
			long time = Long.valueOf(name);
			Trip trip = SqliteAccess.loadTrip(file, time);
			List<Trace> accelerometer = PreProcess.exponentialMovingAverage(trip.accelerometer_, -1);
			
			
			int changes = detectOrientationChange(accelerometer);
			Log.log(TAG, file, changes);
			//double distortion = SensorCluster.calculateClusterVariance(accelerometer);
			//break;
			
			/*
			Trace ntr = new Trace(1);
			ntr.time = trip.time_;
			ntr.values[0] = changes;
			
			res.add(ntr);
			*/
			
			if(changes != 0 || accelerometer.size() < 600) {
				continue;
			}
			double var = SensorCluster.calculateClusterVariance(accelerometer.subList(0, 30*20));
			Trace ntr = new Trace(1);
			ntr.time = trip.time_;
			ntr.values[0] = var;
			res.add(ntr);
		}
		ReadWriteTrace.writeFile(res, Constants.kAlterSenseOutput.concat("evaluation/wild/variances.dat"));
	}
	
	
	public static int detectOrientationChange(List<Trace> accelerometer) {

		int start = 0;
		SensorCluster cluster = new SensorCluster();
		cluster.initCenter(accelerometer.get(start));

		List<Trace> buffer = new LinkedList<Trace>();
		
		List<Trace> res = new ArrayList<Trace>();
		int counter = 0;
		boolean transition = false;
		for(int i = start + 1; i < accelerometer.size(); ++i) {
			Trace cur = accelerometer.get(i);
			double tocluster = cluster.distanceToCenter(cur);
			
			buffer.add(cur);
			if(buffer.size() >= 30) buffer.remove(0);						
			double m2 = SensorCluster.calculateClusterVariance(buffer);
			
			if(m2 > Constants.kOrientationChangeVarianceThreshold) {
				if(transition == false) {
					//Log.log(TAG, cur.time, "start");
				}
				transition = true;
			} else {
				if(transition == true) {
					//Log.log(TAG, cur.time, m2);
					transition = false;
					counter++;
				}
			}
			cluster.addAccelerometer(cur);			
			Trace ntr = new Trace(1);
			ntr.time = cur.time;
			ntr.values[0] = m2;
			res.add(ntr);
		}
		return counter;

	}
}
