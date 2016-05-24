package main;

import java.util.List;

import tracereplay.DirectoryWalker;
import tracereplay.ReadWriteTrace;
import utility.Constants;
import utility.Log;
import utility.Trace;

public class Projection {
	
	private static final String TAG = "Projection";
	private static final String root = Constants.kHome + "/backups/projects/obd/data/rawdat";
	
	
	private static final String lei = root + "/bozhao/urban";

	public static void start() {
		Log.log(TAG, lei);
		List<String> files = DirectoryWalker.getFolders(lei);
		
		for(int i = 0; i < files.size(); ++i) {
			String file = files.get(i);
			Log.log(TAG, file);
			List<Trace> gps = ReadWriteTrace.readFile(file.concat("/gps.dat"), 3);
			if(gps.size() == 0) continue;
			//Log.log(gps.get(0).toJson());
			double score = rating(gps);
			//break;
			if(score < 0.0) {
				Log.error(TAG, file);	
			}
		}
	}
	
	 
	public static double rating(List<Trace> gps) {
		int sz = gps.size();
	    int counter_ = 0;
	    Trace lastTrace_ = null;
	    double lastSpeed_ = -1.0;
	    double score_ = 10.0;
		for(int i = 0; i < sz; ++i) {
			Trace trace = gps.get(i);
			if(lastTrace_ == null) {
	            lastTrace_ = trace;
	            continue;
	        }
	        double time = trace.time - lastTrace_.time;
	        double curSpeed = Trace.distance(lastTrace_, trace)/(time/1000.0);
	        if(lastSpeed_ == -1.0) {
	            lastSpeed_ = curSpeed;
	            continue;
	        } else if(curSpeed == 0.0) {
	        	continue;
	        } else {
	        	counter_++;
	        }
	        
	        double a = (curSpeed - lastSpeed_)/(time/1000.0);

	        lastSpeed_ = curSpeed;
	        lastTrace_ = trace;
	        if(a < -2.0) {
	            double curscore = 3.0 - Math.min(3.0, Math.abs(a));
	            score_ = (score_ * (counter_ - 1) + curscore * 10.0)/counter_;
	        } else {
	            score_ = (score_ * (counter_ - 1) + 10.0)/counter_;
	        }
	       // Log.log(curSpeed, a, score_);
	    }
		//Log.log(TAG, score_);
		return score_;
	}

}
