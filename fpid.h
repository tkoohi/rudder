#include <RobustControl>;


public class fpid{
	private int KeyPressed = 0;
	private  double errPrev = 0;
	private  RobustControl rc;
	    
	
	public fpid() {
		rc = new RobustControl();

	}

	/*public double rudder_fpid(double headingerror) {
		double ruddersign = rc.changerudder(headingerror);
		return ruddersign;

	}*/
	
	public void applyHardControl(int carX, int carY, int x){
		
		rc.setErrPrev(errPrev);
		double a = rc.action(carX, carY, x );
		errPrev = rc.getErrPrev();
	}
	
	
		public void setKeyPressed(int keyPressed) {
			KeyPressed = keyPressed;
		}

		public int getKeyPressed() {
			return KeyPressed;
		}

		
	
}
