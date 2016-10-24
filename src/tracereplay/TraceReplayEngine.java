package tracereplay;

import java.util.List;

import utility.Trace;

public class TraceReplayEngine {
	/**
	 * emulate real time running program
	 * @param input
	 * @param detector
	 */
	public static void traceReplay(List<List<Trace> > input, RealTimeBehaviorDetector detector) {
		int num = input.size();
		int index[] = new int[num];
		for(int i = 0; i < num; ++i) {
			index[i] = 0;
		}
		while(true) {
			int cur = -1;
			long time = Long.MAX_VALUE;
			for(int i = 0; i < num; ++i) {
				int j = index[i];
				if(j >= input.get(i).size()) {
					continue;
				}
				Trace trace = input.get(i).get(j);
				if(trace.time < time) {
					time = trace.time;
					cur = i;
				}
			}
			if(-1 == cur) {
				break;
			}
			detector.processTrace(input.get(cur).get(index[cur]));
			index[cur] ++;
		}
	}
	
}
