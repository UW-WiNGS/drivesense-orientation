package utility;


public class PairDouble {

	public double x;
	public double y;
	
	public PairDouble() {
		x = 0;
		y = 0;
	}
	
	public PairDouble(double x, double y) {
		this.x = x;
		this.y = y;
	}
	
	public String toString() {
		String res = String.valueOf(x) + Constants.kOutputSeperator + String.valueOf(y);
		return res;
	}

}
