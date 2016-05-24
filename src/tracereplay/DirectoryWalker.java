package tracereplay;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import utility.Log;

public class DirectoryWalker {

	private final static String TAG = "DirectoryWalker";
	private static List<String> files_;
	
	/*rules to match files:*/
	private static List<String> names_ = null;
	private static List<String> postfix_ = null;
	
	
	private static boolean hasName(String name) {
		if(names_ != null) {
			for(String fname: names_) {
				if(fname.compareTo(name)==0)
					return true;
			}
			return false;
		} else {
			return true;
		}
	}
	private static boolean hasPostfix(String name) {
		if(postfix_ != null) {
			for(String post: postfix_) {
				if(name.endsWith(post))
					return true;
			}
			return false;
		} else {
			return true;
		}
	}
	private static boolean record(String name) {
		if(hasPostfix(name) && hasName(name))
			return true;
		return false;
	}
	
	/**
	 * 
	 * set the rules to match the files
	 * @param names
	 * @param posts
	 */
	public static void setRules(List<String> names, List<String> posts) {
		if(names!=null && names.size() > 0)
			names_ = new ArrayList<String>(names);
		if(posts!=null && posts.size() > 0)
			postfix_ = new ArrayList<String>(posts);
	}
	
	/**
	 * get all the files under the directory of root
	 * 
	 * */
	public static List<String> recursive(String root) {
		files_ = new ArrayList<String>();
		File cur = new File(root);
		if(cur.exists()) {
			walk(cur.getAbsoluteFile());	
		} else {
			Log.log(root + " does not exists!");
		}
		return files_;
	}
	
	public static void createFolder(String directory, String name) {
		Log.log(TAG, "create folder:" + directory + name);
		File dir = new File(directory.concat(name));
		if(dir.exists()) {
			Log.log(TAG, directory + name + " already exists");
			return;
		}
		if(true != dir.mkdirs()) {
			Log.log(TAG, "create folder error: folder:" + directory + " file name:" + name);
		}
	}
	
	/**
	 * 
	 * get all the files under directory
	 * @param directory
	 * @return
	 */
	public static List<String> getFileNames(String directory) {
		File cur = new File(directory);
		List<String> res = new ArrayList<String>();
		File[] list = cur.listFiles();
		for ( File f : list ) {
			if ( f.isFile() ) {    
				res.add(f.getName());
			}
		}
		return res;
	}
	
	/**
	 * 
	 * get all the folders under directory
	 * @param directory
	 * @return
	 */
	public static List<String> getFolders(String directory) {
		File cur = new File(directory);
		List<String> res = new ArrayList<String>();
		File[] list = cur.listFiles();
		for ( File f : list ) {
			if ( f.isDirectory() ) {    
				res.add(f.getAbsolutePath());
			}
		}
		return res;
	}
	
	private static void walk(File root) {
		
		File[] list = root.listFiles();

		if (list == null) return;

		for ( File f : list ) {
			if ( f.isDirectory() ) {    
				walk(f);   
				//System.out.println( "Dir:" + f.getAbsoluteFile() );    
			}
            else {
            	if(record(f.getName())) {
            		files_.add(f.getAbsolutePath());
            	 	//Log.log(f.getName());
                    //System.out.println( "File:" + f.getAbsoluteFile() );
               
            	}
            }
        }
	    
	}
}
