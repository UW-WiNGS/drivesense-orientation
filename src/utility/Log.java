package utility;

public class Log {
	
	private final static String kSeperator = ",";
	
	public static void d(Object obj) {
		System.out.println(obj);
	}
	
	public static void d(Object src, Object obj) {
		System.out.println(src + kSeperator + obj);
	}
	
	public static void d(Object src, Object obj, Object obj1) {
		System.out.println(src + kSeperator + obj  + kSeperator + obj1);
	}
	
	
	public static void error(Object src, Object obj) {
		System.err.println(src + kSeperator + obj);
	}
	
	public static void error(Object src, Object obj0, Object obj1) {
		System.err.println(src + kSeperator + obj0 + kSeperator + obj1);
	}
}
