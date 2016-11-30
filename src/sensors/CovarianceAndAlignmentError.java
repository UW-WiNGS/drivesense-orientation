package sensors;

import gpsevaluation.GPSAbstraction;
import io.DirectoryWalker;
import io.ReadWriteTrace;
import io.SqliteAccess;

import java.util.ArrayList;
import java.util.List;

import utility.Constants;
import utility.Formulas;
import utility.Log;
import utility.PreProcess;
import utility.Trace;
import utility.Trip;

public class CovarianceAndAlignmentError {
	private final static String TAG = "CovarianceAndAlignmentError";
	
	public static void start() {
		calculateDistortionAndError();
		//correlationBetweenCovarianceAndError();
		
		//outputVariousOrientation();
	}
	
	private static void outputVariousOrientation() {
		//String input = Constants.kUncontrol.concat("controlled/pocket/");
		String input = Constants.kUncontrol.concat("controlled/holding/");
		
		String output = Constants.outputPath.concat("orientation/varies/");
		List<String> files = DirectoryWalker.getFilePaths(input);
		
		int counter = 0;
		for(String file: files) {
			counter++;
			String name = file.substring(file.length() - 16, file.length() - 3);
			long time = Long.valueOf(name);
			Trip trip = SqliteAccess.loadTrip(file, time);
			List<Trace> accelerometer = PreProcess.exponentialMovingAverage(trip.accelerometer_, -1);
			//
			if(counter == 2) {
				ReadWriteTrace.writeFile(accelerometer, output.concat(name + ".dat"));
			    break;
			}
			
			double distortion = OrientationChangeDetection.clusterDistortion(accelerometer);
			Log.log(TAG, distortion);
			//break;
		}
	}
	
	
	/**
	 * TODO: cluster distortion, to an array 1 - 10, to see the sum errors
	 */
	private static void calculateDistortionAndError() {
		String outfolder = Constants.outputPath.concat("alignment/distortion/");
		List<String> folders = DirectoryWalker.getFolders(Constants.datPath);
		List<Trace> output0 = new ArrayList<Trace>();
		List<Trace> output1 = new ArrayList<Trace>();

		int counter = 0;
		
		
		for(String directory: folders) {
			List<Trip> trips = ReadWriteTrace.loadTrips(directory.concat("/urban"));
			List<Trip> highway = ReadWriteTrace.loadTrips(directory.concat("/highway"));
			trips.addAll(highway);
			
			for(Trip trip: trips) {
				Log.log(TAG, trip.path);
				List<Trace> accelerometer = processTrip(trip);
				if(accelerometer.size() < 10) continue;
				//double distortion = OrientationChangeDetection.clusterDistortion(accelerometer);
				List<Trace> distortions = OrientationChangeDetection.trackClusterDistortion(accelerometer);
				Trace distor = PreProcess.getTraceAt(distortions, 30*1000);
				if(distor == null) continue;
				double distortion = distor.values[2];
				
				Trace error = CoordinateAlignment.alignmentPerformance(trip);
				if(error == null) continue;
				Trace tr = new Trace(error.dim + 1);
				tr.values[0] = distortion;
				for(int i = 0; i < error.dim; ++i) {
					tr.values[i + 1] = error.values[i];
				}
				if(error.values[error.dim - 3] > 3.0) {
					continue;
				}
				
				Log.log(TAG, tr.toJson());
				
				if(distortion < 0.15) {
					output0.add(tr);			
				} else {
					output1.add(tr);
				}
			}
		}	
		ReadWriteTrace.writeFile(output0, outfolder.concat("distortion_error0.dat"));	
		ReadWriteTrace.writeFile(output1, outfolder.concat("distortion_error1.dat"));	
	}

	private static List<Trace> processTrip(Trip trip) {	
		List<Trace> accelerometer = PreProcess.exponentialMovingAverage(trip.accelerometer_, -1);
		List<Trace> gyroscope = PreProcess.exponentialMovingAverage(trip.gyroscope_, -1);
		List<Trace> gps = PreProcess.exponentialMovingAverage(GPSAbstraction.wrapperGPS(trip.gps_elevation_), 3);
		List<Trace> res = new ArrayList<Trace>();		
		int sample_rate = 1;
		for(int i = 0; i < gps.size(); ++i) {
			Trace cur = gps.get(i);
			if(Math.abs(cur.values[3]) < 0.5) {
				Trace curacc = PreProcess.getTraceAt(accelerometer, cur.time);
				if(curacc == null) {
					continue;
				}
				res.add(curacc);
			}
			
		}
		return res;
	}
	
	
	/**
	 * Too small and useless ??? !!!
	 */
	private static void correlationBetweenCovarianceAndError() {
		String input = Constants.outputPath.concat("alignment/data/distortion_error.dat");
		int dim = 11;
		List<Trace> data = ReadWriteTrace.readFile(input, dim);
		List<Trace> tmp = new ArrayList<Trace>();
		
		int group = 2;
		double accuracy[] = new double[group];
		int counter[] = new int[group];
		
		for(int i = 0; i < data.size(); ++i) {
			Trace cur = data.get(i);
		
			if(cur.values[0] > 1.0) {
				continue;
			}
			
			
			Trace ntr = new Trace(1);
			ntr.copyTrace(cur);
			tmp.add(ntr);
			
			int index = (int)(ntr.values[0] * group) % group;
			accuracy[index] += ntr.values[9];
			counter[index] ++;
		}
		
		for(int i = 0; i < group; ++i) {
			Log.log(TAG, accuracy[i]/counter[i]);
		}
		
		/*
		for(int i = 0; i < dim - 1; ++i) {
			double corr = Formulas.linear_correlation(tmp, 0, i + 1);
			Log.log(i, corr);
		}
		*/
		
	}
	
}
