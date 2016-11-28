package sensors;

import gpsevaluation.GPSAbstraction;
import io.DirectoryWalker;
import io.ReadWriteTrace;

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
		//calculateDistortionAndError();
		correlationBetweenCovarianceAndError();
	}
	
	private static void calculateDistortionAndError() {
		String outfolder = Constants.outputPath.concat("alignment/distortion/");
		List<String> folders = DirectoryWalker.getFolders(Constants.datPath);
		List<Trace> output = new ArrayList<Trace>();
		String type = "urban";
		int counter = 0;
		for(String directory: folders) {
			String folder = directory.concat("/" + type);
			List<Trip> trips = ReadWriteTrace.loadTrips(folder);
			for(Trip trip: trips) {
				Log.log(TAG, trip.path);
				List<Trace> accelerometer = processTrip(trip);
				if(accelerometer.size() < 10) continue;
				double distortion = OrientationChangeDetection.clusterDistortion(accelerometer);
				Trace error = CoordinateAlignment.alignmentPerformance(trip);
				if(error == null) continue;
				Trace tr = new Trace(error.dim + 1);
				tr.values[0] = distortion;
				for(int i = 0; i < error.dim; ++i) {
					tr.values[i + 1] = error.values[i];
				}
				Log.log(TAG, tr.toJson());
				output.add(tr);				
			}
		}	
		ReadWriteTrace.writeFile(output, outfolder.concat("distortion_error.dat"));	
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
	
	private static void correlationBetweenCovarianceAndError() {
		String input = Constants.outputPath.concat("alignment/data/error.dat");
		int dim = 11;
		List<Trace> data = ReadWriteTrace.readFile(input, dim);
		List<Trace> tmp = new ArrayList<Trace>();
		
		double accuracy[] = new double[10];
		for(int i = 0; i < data.size(); ++i) {
			Trace cur = data.get(i);
			if(cur.values[0] > 1.0 || cur.values[8] > 1.2) {
				continue;
			}
			Trace ntr = new Trace(1);
			ntr.copyTrace(cur);
			tmp.add(ntr);			
		}
		
		/*
		for(int i = 0; i < dim - 1; ++i) {
			double corr = Formulas.linear_correlation(tmp, 0, i + 1);
			Log.log(i, corr);
		}
		*/
	}
	
}
