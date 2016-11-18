package utility;

import java.util.ArrayList;
import java.util.List;

import utility.Constants;

public class Pattern {
	
	public static final String kAcceleration = "acceleration";
//	public static final String kConstant = "constant";
	public static final String kBrake = "brake";
	public static final String kStop = "stop";
	public static final String kTurn = "turn";
	public static final String kLanechange = "lanechange";
	
	
	public static final String kToLeft = "toleft";
	public static final String kToRight = "toright";
	
	public static final String kbump = "bump";
	
	/*start time and end time (in MillSec)*/
	public long start = -1;
	public long end = -1;
	
	public int start_index = -1;
	public int end_index = -1;
		
	public double accumulated_change = 0.0;
	/*start and end index*/
	public double score = -1.0;
	
	public String type = "";
	
	
	public void setTimeInterval(long s, long e) {
		start = s;
		end = e;
	}
	public Pattern() {
		start = -1;
		end = -1;
	}
	
	public String toString()
	{
		StringBuilder sb = new StringBuilder();
		sb.append("Current Pattern: \n ");
		sb.append("type:" + type).append("\n");
		sb.append("\tscore:").append(score).append("\n");
		sb.append("\tstart_index:").append(this.start_index).append("\n\tend_index: ").append(this.end_index).append("\n");		
		sb.append("\tstart:").append(this.start).append("\n\tend: ").append(this.end).append("\n");
		return sb.toString();
	}
	
	public void getPattern(String line) {
		String[] res = line.split(Constants.kInputSeperator);
		end = Long.parseLong(res[0]);
		start = end - 30* 1000;
		
		score = Double.parseDouble(res[1]);
		type = res[2];
	}
	
	
	//merge overlapping intervals into one interval. Score of the merged interval will be average of scores of the overlapping intervals 
	public static List<Pattern> reduceOverlapIntervals(List<Pattern> intervals) {
		List<Pattern> res = new ArrayList<Pattern>();
		int sz = intervals.size();
		if(0==sz) return intervals;
		

		Pattern last = new Pattern();
		last = intervals.get(0);
		double avg = last.score;
		int count = 1;
		
		for(int i = 1; i < sz; ++i) {
			Pattern cur = intervals.get(i);
			if(cur.start <= last.end) {
				last.end = cur.end;
				avg = avg + cur.score;
				count++;
			} else {
				avg = avg/count;
				last.score = avg;
				res.add(last);
				last = new Pattern();
				last = cur;
				avg = last.score;
				count = 1;
			}
		}
		res.add(last);
		return res;
	}
	
}
