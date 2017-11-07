public class RobustControl{
	

	RuleBase TrackingRulebase1;
	RuleBase RudderCRulebase2;
	float errPrev;
	    
	    //declare input sets
     T1MF_Triangular Ne,Ze,Pe, Neprim,Zeprim,Peprim, Nheading, Zheading, Pheading, NBheading, PBheading;
	
	    
	   
	
	/** Creates a new instance of FuzzyController */
    
	public  RobustControl() {
		
		setupRulebase();

	}
      
	/**
	 * 
	 * @param x0
	 * @param y0
	 * @param x
	 * @param errPrev2 
	 * @param errPrev 
	 * @return Defuzzification of Singletons
	 */
	public float action(int x0, int y0, int x) {
		new LookUpT();
		LineDist ld = new LineDist();
		ld.setErrPrev(getErrPrev());
		ld.computeErr(x0, y0, x, 0, LookUpT.Tbase);
					
	    /*
	     * set inputs to input sets
	     */
        Ne.setInput(ld.e);
        Ze.setInput(ld.e);
        Pe.setInput(ld.e);
        //System.out.println("e = "+ ld.e);
        
        
        Neprim.setInput(ld.ePrim);
        Zeprim.setInput(ld.ePrim);
        Peprim.setInput(ld.ePrim);
        //System.out.println("ePrim = "+ ld.ePrim);
        

        float a = TrackingRulebase1.heightSingDefuzzification();
        
        //System.out.println("Fuzzy Tracking Output = "+ round(a));
        setErrPrev(ld.getErrPrev());
        return round(a);
        
    }
	
	
	//change rudder
	/**
	 * 
	 * @param headingerror
     * @return Defuzzification of Triangle
	 */
	/*public double changerudder(double headingerror) {
		Nheading.setInput(headingerror);
		Zheading.setInput(headingerror);
		Pheading.setInput(headingerror);
       

        double a = RudderCRulebase2.heightSingDefuzzification();
        
        System.out.println("Fuzzy Rudder Output = "+ Math.round(a));
        return Math.round(a);
		
	}*/
    
    private void setupRulebase()
    {
        /*
         * input Membership Functions
         */
    	// error
        Ne = new T1MF_Triangular("Ne",-200.0,-50.0,0.0);          
        Ze = new T1MF_Triangular("Ze",-50.0,0.0,50.0);    
        Pe = new T1MF_Triangular("Pe",0.0,50.0,200.0);          

        // error prim
        Neprim = new T1MF_Triangular("Neprim",-30.0,-10.0,0.0);           
        Zeprim = new T1MF_Triangular("Zeprim",-10.0,0.0,10.0);        
        Peprim = new T1MF_Triangular("Peprim",0.0,10.0,30.0);              
        
        // heading error
        NBheading = new T1MF_Triangular("NBheading",-360.0,-180.0,-90.0);
        Nheading = new T1MF_Triangular("Nheading",-180.0,-90.0,0.0);
        Zheading = new T1MF_Triangular("Zheading",-90.0,0.0,90.0);    
        Pheading = new T1MF_Triangular("Pheading",0.0,90.0,180.0);
        PBheading = new T1MF_Triangular("PBheading",90.0,180.0,360.0);

        
        //output Membership Functions
        T1MF_Singular goHardLeft = new T1MF_Singular("Hard Left",-8.0);   //turn hard left
        T1MF_Singular goLeft = new T1MF_Singular("left",-4.0);          //turn left
        T1MF_Singular goStraight = new T1MF_Singular("straight",1.0);   //go straight
        T1MF_Singular goRight = new T1MF_Singular("right",4.0);          //turn right
        T1MF_Singular goHardRight = new T1MF_Singular("Hard Right",8.0);    //turn hard right
        
        // rudder command
        T1MF_Singular NHardrudder = new T1MF_Singular("NHardrudder",-8.0);
        T1MF_Singular Nrudder = new T1MF_Singular("Nrudder",-4.0);    
        T1MF_Singular Zrudder = new T1MF_Singular("Zrudder",1.0);
        T1MF_Singular Prudder = new T1MF_Singular("Prudder",4.0);    
        T1MF_Singular PHardrudder = new T1MF_Singular("PHardrudder",8.0);    



        
        //setup an array with our ruleset 1
        Rule[] ruleset1 = new Rule[9];
        ruleset1[0] = new Rule(new T1MF_Triangular[]{Ne, Neprim}, goHardRight);    
        ruleset1[1] = new Rule(new T1MF_Triangular[]{Ze, Neprim}, goRight);       
        ruleset1[2] = new Rule(new T1MF_Triangular[]{Pe, Neprim}, goHardRight);
        ruleset1[3] = new Rule(new T1MF_Triangular[]{Ne, Zeprim}, goRight);
        ruleset1[4] = new Rule(new T1MF_Triangular[]{Ze, Zeprim}, goStraight);
        ruleset1[5] = new Rule(new T1MF_Triangular[]{Pe, Zeprim}, goLeft);
        ruleset1[6] = new Rule(new T1MF_Triangular[]{Ne, Peprim}, goHardLeft);
        ruleset1[7] = new Rule(new T1MF_Triangular[]{Ze, Peprim}, goLeft);
        ruleset1[8] = new Rule(new T1MF_Triangular[]{Pe, Peprim}, goHardLeft);
        
        //setup an array with our ruleset 2
        /*Rule[] ruleset2 = new Rule[5];
        ruleset2[0] = new Rule(new T1MF_Triangular[]{NBheading}, PHardrudder);        
        ruleset2[1] = new Rule(new T1MF_Triangular[]{Nheading}, Prudder);        
        ruleset2[2] = new Rule(new T1MF_Triangular[]{Zheading}, Zrudder);        
        ruleset2[3] = new Rule(new T1MF_Triangular[]{Pheading}, Nrudder);        
        ruleset2[4] = new Rule(new T1MF_Triangular[]{PBheading}, NHardrudder);*/        

        
        //setup actual rulebases
        TrackingRulebase1 = new RuleBase();
        TrackingRulebase1.addRules(ruleset1);
        
        /*RudderCRulebase2 = new RuleBase();
        RudderCRulebase2.addRules(ruleset2);*/

}

	public void setErrPrev(double errPrev) {
		this.errPrev = errPrev;
	}

	public float getErrPrev() {
		return errPrev;
	}

	
}
