//#include "Representations/Modeling/ObstacleModel.h"
//#include "BehaviorControl/PathPlannerProvider/PathPlannerProvider.h"
//#include "Tools/Modeling/Obstacle.h"

  //int count=1;
 
option(Supporter2)
{
  std::vector<Obstacle> o=theObstacleModel.obstacles;
	  //std::vector<Teammate>t = theTeamData.teammates;
  const Vector2f weizhi = Transformation::fieldToRobot(theRobotPose, Vector2f(3300,0));//3300,相对fieldToRobot
  const Vector2f weizhi2 = Transformation::fieldToRobot(theRobotPose, Vector2f(1000,800));//3300,相对fieldToRobot
  const Vector2f weizhi3 = Transformation::fieldToRobot(theRobotPose, Vector2f(1500,0));//3300,相对fieldToRobot
  const Vector2f weizhi4 = Transformation::fieldToRobot(theRobotPose, Vector2f(1000,-800));//3300,相对fieldToRobot
  const Vector2f weizhi5 = Transformation::fieldToRobot(theRobotPose, Vector2f(-1500,0));
  const Vector2f ball = theBallModel.estimate.position;
  const Vector2f globalBall = Transformation::robotToField(theRobotPose,ball);
  const Vector2f globalrobot = Transformation::robotToField(theRobotPose, Vector2f(0,0));//3300，绝对robotToField
  const float globalrobot_x = theRobotPose.translation.x();
  const float globalrobot_y = theRobotPose.translation.y();
  const float globalrobot_theta = theRobotPose.rotation;
  //std::vector<Teammate> t = theTeamData.teammates;
  static Vector2f ball_relative = Vector2f(1000.f,0.f);
  static Vector2f globalBall1 = Vector2f(1000.f,0.f);
  //double deltadist =std::sqrt((t[].theRobotPose.x()-globalrobot_x)^2+(player.theRobotPose.x()-globalrobot_y)^2)
  //const float xiangdui_x=player.theRobotPose.x()-globalrobot_x)     
  static int flag;
  static int qufen=0;
  initial_state(start)//开始
  {
    transition
    {
      if(state_time > 1000)
        goto turnToBall;
	
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
	  //LookAround();
      Stand();
    }
  }
    
  state(searchForBall)//找球
  {
    transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen < 300)
        goto turnToBall;
	
	  if(state_time>8000&&qufen==0)
		goto turntoit;
	 if(state_time>8000&&qufen==1)
		goto turntoit1;
	  if(state_time>8000&&qufen==2)
		goto turntoit2;
		if(state_time>8000&&qufen==3)
		goto turntoit3;
	
	
//	  for(int i=0;i<t.size();i++)
//	{
//	 if(t[i].number==2 && theFrameInfo.getTimeSince(t[i].theBallModel.timeWhenLastSeen)<500)
//	  {
//	  globalBall1 = Transformation::robotToField(t[i].theRobotPose, t[i].theBallModel.estimate.position);
//	  ball_relative = Transformation::fieldToRobot(theRobotPose, globalBall1);
//	  goto turntoball;
//	  }
//	  
//	}
			
    }
    action
    {
      //HeadControlMode(HeadControl::lookForward);
	  //LookAround();
      WalkAtRelativeSpeed(Pose2f(1.f, 0.f, 0.f));
    }
  }
//    state(turntoball)//转向球
//  {
//	  
//	 transition
//    {
//		
//
//      if(std::abs(ball_relative.angle()) < 3_deg)
//        goto gotoward_ball;
//	  if(state_time > 4000)
//		goto gotoward_ball;
//	
//    }
//    action
//    {
//      HeadControlMode(HeadControl::lookForward);
//      WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(ball_relative.angle(), 0.f, 0.f));
//    } 
//	  
//  }
  
//  state(gotoward_ball)//走向球
//  {
//	  transition
//    {
//      if(theLibCodeRelease.timeSinceBallWasSeen < 300)
//        goto turnToBall;
//      if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut
//	  &&(std::sqrt(ball_relative.x()*ball_relative.x()+ball_relative.y()*ball_relative.y())<200))
//        goto searchForBall;  	
//
//    }
//    action
//    {
//    //  LookAround();
//	  WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(0_deg, ball_relative.x(), ball_relative.y()));
//	
//    }
//	  
//  }
  
  state(turntoit)//
  {
	  
	 transition
    {
		

      if(std::abs(weizhi2.angle()) < 3_deg)
        goto gotoward;
	  if(state_time > 4000)
		goto gotoward;
	
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
      WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(weizhi2.angle(), 0.f, 0.f));
    } 
	  
  }
    state(turntoit1)//
  {
	  
	 transition
    {
		

      if(std::abs(weizhi3.angle()) < 3_deg)
        goto gotoward1;
	  if(state_time > 4000)
		goto gotoward1;
	
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
      WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(weizhi3.angle(), 0.f, 0.f));
    } 
	  
  }
  state(turntoit2)//
{
	  
	 transition
    {
		

      if(std::abs(weizhi4.angle()) < 3_deg)
        goto gotoward2;
	  if(state_time > 4000)
		goto gotoward2;
	
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
      WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(weizhi4.angle(), 0.f, 0.f));
    } 
	  
  }
  state(turntoit3)//
{
	  
	 transition
    {
		

      if(std::abs(weizhi5.angle()) < 3_deg)
        goto gotoward3;
	  if(state_time > 4000)
		goto gotoward3;
	
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
      WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(weizhi5.angle(), 0.f, 0.f));
    } 
	  
  }
  state(gotoward)//
{
	  transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen < 300)
        goto turnToBall;
      if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut
	  &&(std::sqrt(weizhi2.x()*weizhi2.x()+weizhi2.y()*weizhi2.y())<200))
        goto searchForBall;  	

    }
    action
    {
	  qufen=1;
    //  LookAround();
	  WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(0_deg, weizhi2.x(), weizhi2.y()));
	
    }
	  
  }
    state(gotoward1)//
{
	  transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen < 300)
        goto turnToBall;
	  if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut
	   &&(std::sqrt(weizhi3.x()*weizhi3.x()+weizhi3.y()*weizhi3.y())<200))
        goto searchForBall;	

    }
    action
    {
	 qufen=2;
      LookAround();
	  WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(0_deg, weizhi3.x(), weizhi3.y()));
	
    }
	  
  }
  state(gotoward2)//
{
	  transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen < 300)
        goto turnToBall;
  if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut
   &&(std::sqrt(weizhi4.x()*weizhi4.x()+weizhi4.y()*weizhi4.y())<200))
        goto searchForBall;  	

    }
    action
    {
		qufen=3;
     // LookAround();
	  WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(0_deg, weizhi4.x(), weizhi4.y()));	
    }
	  
  }
  state(gotoward3)//
{
	  transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen < 300)
        goto turnToBall;
  if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut
   &&(std::sqrt(weizhi5.x()*weizhi5.x()+weizhi5.y()*weizhi5.y())<200))
        goto searchForBall;  	

    }
    action
    {
		qufen=0;
     // LookAround();
	  WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(0_deg, weizhi5.x(), weizhi5.y()));	
    }
	  
  }
state(turnToBall)//
  {
    transition
    {			
        if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
        goto searchForBall;
      if(std::abs(theBallModel.estimate.position.angle()) < 5_deg)
        goto walkToBall;	
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
      WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(theBallModel.estimate.position.angle(), 0.f, 0.f));
    }
  }
  state(walkToBall)//走向球
{
    transition
	
    {
		
		/*for(int k=0;k<t.size();k++)
		{
		  if(t[k].number==2&&t[k].mateType==Teammate::BHumanRobot&&t[k].theBallModel.estimate.position.norm()<ball.norm())	
		  {
		goto dangren;   
			
		  }
		}
		*/
	    if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
          goto searchForBall;
		  
		  
		   for(int j=0;j<o.size();j++)
	    {
		
		if( 
		o[j].type==Obstacle::opponent
	    &&o[j].center.x()>0 && o[j].center.x() <800
		&& o[j].center.y() >-200&& o[j].center.y()<250
		&&theBallModel.estimate.position.norm() < 500.f
		&&(globalrobot.x()-globalBall.x())<0
		)
		
	    {
			goto alignToGoal_side;
	    }
		} 
		  
		if(theBallModel.estimate.position.norm() < 500.f&&(globalrobot.x()-globalBall.x())<0)
          goto alignToGoal;
		   
		if(theBallModel.estimate.position.norm() < 250.f&&(globalrobot.x()-globalBall.x())>0)
		  goto alignToGoal_change;

    }
    action
    {
      //HeadControlMode(HeadControl::lookForward);
	 LookAtBall();
	  //WalkToTarget(Pose2f(60.f, 60.f, 60.f),  Pose2f(0_deg,theBallModel.estimate.position.x(), theBallModel.estimate.position.y()));
      WalkToTarget(Pose2f(50.f, 50.f, 50.f), theBallModel.estimate.position);
	}
  }
  state(dangren)//挡人
{
	transition
	{

	if(state_time>5000)
	{	
	goto searchForBall;
	}

		
	}
	
	
	action
	   {
		    HeadControlMode(HeadControl::lookForward);

		   int i;
      for(  i=0;i<o.size();i++)
        {
		if(o[i].type==Obstacle::opponent&&weizhi.norm()>1600)
         WalkToTarget(Pose2f(80.f, 80.f, 80.f), o[i].center);   
        }
		
	
	   }
	
}
   state(alignToGoal_change)//朝向球门_改
  {
    transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
        goto searchForBall;
      if(std::abs(theLibCodeRelease.angleToGoal) < 10_deg)// && std::abs(theBallModel.estimate.position.y()) < .f)
        goto alignBehindBall;
	int number=0; 
      for( size_t i=0;i<o.size();i++)
        {
		if(o[i].center.x()>0&&o[i].center.x()<300&&abs(o[i].center.y())<250&&(o[i].type==Obstacle::teammate))//||o[0].type==Obstacle::someRobot||o[0].type==Obstacle::fallenSomeRobot))
            number++;
        }
		if(number>0)
		{   
            goto walkToLeftTargets; 
		}
		
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
     // WalkToTarget(Pose2f(100.f, 100.f, 100.f), Pose2f(theLibCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 400.f, theBallModel.estimate.position.y()));
	  WalkToTarget(Pose2f(80.f, 80.f, 80.f), Pose2f(ball.angle(), theBallModel.estimate.position.x() - 220.f, 100.f));
	  
    }
  }
   state(alignToGoal)//朝向球门
  {
    transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
        goto searchForBall;
		
      if(std::abs(theLibCodeRelease.angleToGoal) < 10_deg && std::abs(theBallModel.estimate.position.y()) < 100.f)
        goto alignBehindBall;
//	 if(std::abs(theLibCodeRelease.angleToGoal) < 10_deg && std::abs(theBallModel.estimate.position.y()) < 100.f&&globalrobot_y<0)
  //      goto alignBehindBall_right;
		/*
	int number=0; 
      for( size_t i=0;i<o.size();i++)
        {
		if(o[i].center.x()>0&&o[i].center.x()<300&&abs(o[i].center.y())<250&&(o[i].type==Obstacle::teammate))//||o[0].type==Obstacle::someRobot||o[0].type==Obstacle::fallenSomeRobot))
            number++;
        }
		if(number>0)
		{   
            goto walkToLeftTargets; 
		}
		*/
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
	  WalkToTarget(Pose2f(35.f, 35.f, 35.f), Pose2f(theLibCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 400.f, theBallModel.estimate.position.y()));
	  //WalkToTarget(Pose2f(100.f, 0.5f, 50.f), Pose2f(theLibCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 400.f, theBallModel.estimate.position.y()-400.f));	  
    }
  }
    state(alignToGoal_side)//走向球门侧边
  {
    transition
    {
		if(globalrobot_y>0)
		flag=0;
		if(globalrobot_y<0)
		flag=1;
		
		
      if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
        goto searchForBall;
		
      if(std::abs(theLibCodeRelease.angleToGoal) < 50_deg && theLibCodeRelease.between(theBallModel.estimate.position.y(), -30.f, 0.f)
	   &&theLibCodeRelease.between(theBallModel.estimate.position.x(), 140.f, 170.f)&&globalrobot_y>0)	
	   {
        goto sidekick;
	   }
	
	if(std::abs(theLibCodeRelease.angleToGoal) < 50_deg && theLibCodeRelease.between(theBallModel.estimate.position.y(), 20.f, 50.f)
	   &&theLibCodeRelease.between(theBallModel.estimate.position.x(), 140.f, 170.f)&&globalrobot_y<0)	
       { 
		   goto sidekick_right;
	   }
		
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
	  if(flag==1)
	  WalkToTarget(Pose2f(35.f, 35.f, 35.f), Pose2f(theLibCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 150.f, theBallModel.estimate.position.y()-30.f));
	  if(flag==0)
	  WalkToTarget(Pose2f(35.f, 35.f, 35.f), Pose2f(theLibCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 150.f, theBallModel.estimate.position.y()+30.f));
    }
  }
     state(alignBehindBall)//对齐球后面
  {
    transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
        goto searchForBall;
		/*
	   if(theLibCodeRelease.between(theBallModel.estimate.position.y(), 20.f, 50.f)
         && theLibCodeRelease.between(theBallModel.estimate.position.x(), 140.f, 170.f)		 
         && std::abs(theLibCodeRelease.angleToGoal) < 2_deg
		 &&std::abs(weizhi.x())<2500)
		  goto dajiao;
		   * */
		   //	int j=0;  
 /*
		    for(int j=0;j<o.size();j++)
	    {
		
		if( 
		 o[j].type==Obstacle::opponent
		&&  0 < o[j].center.x() <280
		&& -250< o[j].center.y()<250
		&& theLibCodeRelease.between(theBallModel.estimate.position.y(), 20.f, 50.f)
		&& theLibCodeRelease.between(theBallModel.estimate.position.x(), 140.f, 170.f)		 
		&& std::abs(theLibCodeRelease.angleToGoal) < 2_deg
		)
	    {
			goto sidekick;
	    }
		}*/
	    if(theLibCodeRelease.between(theBallModel.estimate.position.y(), 20.f, 50.f)
      //   && theLibCodeRelease.between(theBallModel.estimate.position.x(), 140.f, 170.f)	
           && theLibCodeRelease.between(theBallModel.estimate.position.x(), 150.f, 180.f)		 
         && std::abs(theLibCodeRelease.angleToGoal) < 2_deg)
         goto kick;	 
  
		 //  if(theLibCodeRelease.between(theBallModel.estimate.position.y(), 20.f, 50.f)
      //   && theLibCodeRelease.between(theBallModel.estimate.position.x(), 140.f, 170.f)	
          // && theLibCodeRelease.between(theBallModel.estimate.position.x(), 150.f, 180.f)		 
       //  && std::abs(theLibCodeRelease.angleToGoal) < 2_deg&&std::abs(weizhi.x())<2100)
       //  goto kick_pian;	 
  


	}
    
    action
    {
      theHeadControlMode = HeadControl::lookForward;
	//WalkToTarget(Pose2f(15.f, 15.f, 15.f), Pose2f(theLibCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 150.f,        theBallModel.estimate.position.y()-30.f));
	//WalkToTarget(Pose2f(15.f, 15.f, 15.f), Pose2f(theLibCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 170.f,   theBallModel.estimate.position.y()-30.f));
	WalkToTarget(Pose2f(15.f, 15.f, 15.f), Pose2f(theLibCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 170.f,   theBallModel.estimate.position.y()-30.f));
    }
  }
  
  /*
  state(alignBehindBall_right)
  {
    transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
        goto searchForBall;
		
	   if(theLibCodeRelease.between(theBallModel.estimate.position.y(), 20.f, 50.f)
         && theLibCodeRelease.between(theBallModel.estimate.position.x(), 140.f, 170.f)		 
         && std::abs(theLibCodeRelease.angleToGoal) < 2_deg
		 &&std::abs(weizhi.x())<2500)
		  goto dajiao;
		 	
	    if(theLibCodeRelease.between(theBallModel.estimate.position.y(), -30.f, 0.f)
         && theLibCodeRelease.between(theBallModel.estimate.position.x(), 140.f, 170.f)		 
         && std::abs(theLibCodeRelease.angleToGoal) < 2_deg)
         goto kick_right;	   

	}
    
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      WalkToTarget(Pose2f(20.f, 20.f, 20.f), Pose2f(theLibCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 150.f,        theBallModel.estimate.position.y()+30.f));
    }
  }
  


  */

 state(walkToLeftTargets)//走向球左侧
  {
      transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
        goto searchForBall;
	  if(state_time>3000)
		goto turnToBall;
    }
    action
    {
    HeadControlMode(HeadControl::lookForward);

    WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(3_deg, 100.f, 400.f ));
    }
    
  }
   
   state(kick)//踢
  {
    transition
    {
		 
	  
      if(state_time > 3000 || (state_time > 10 && action_done))
        goto start;
   		if(std::abs(weizhi.x())<2000)
	  {
		  goto dajiao;
		  
	  }
	}
    action
    {
		 
     HeadControlMode(HeadControl::lookForward);
      InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(theLibCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 160.f, theBallModel.estimate.position.y() - 55.f));
    }
  }
 /*
   state(kick_pian)
  {
    transition
    {
		 
	  
      if(state_time > 3000 || (state_time > 10 && action_done))
        goto start;
		if(std::abs(weizhi.x())<2000)
	  {
		  goto dajiao;
		  
	  }
		
   
	}
    action
    {
		 
     HeadControlMode(HeadControl::lookForward);
      InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(theLibCodeRelease.angleToGoal+20_deg, theBallModel.estimate.position.x() - 160.f, theBallModel.estimate.position.y() - 55.f));
    }
  }
   */
/*
 state(kick_right)
  {
    transition
    {
		 
	  
      if(state_time > 3000 || (state_time > 10 && action_done))
        goto start;
		if(std::abs(weizhi.x())<1500)
	  {
		  goto dajiao;
		  
	  }
		
	}
    action
    {
		 
     HeadControlMode(HeadControl::lookForward);
      InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::right), Pose2f(theLibCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 160.f, theBallModel.estimate.position.y() - 55.f));
    }
  }
  
  */
 state(sidekick)//侧踢
  {
    transition
    {
		 
	  
      if(state_time > 3000 || (state_time > 10 && action_done))
        goto start;		
	}
    action
    {
		 
     HeadControlMode(HeadControl::lookForward);
	InWalkKick(WalkKickVariant(WalkKicks::sidewardsInner, Legs::left), Pose2f(theLibCodeRelease.angleToGoal-30_deg, theBallModel.estimate.position.x() - 160.f, theBallModel.estimate.position.y() - 85.f));
    }
  }
  
 state(sidekick_right)//侧踢_右
  {
    transition
    {
		 
	  
      if(state_time > 3000 || (state_time > 10 && action_done))
        goto start;		
	}
    action
    {
		 
     HeadControlMode(HeadControl::lookForward);
	InWalkKick(WalkKickVariant(WalkKicks::sidewardsInner, Legs::right), Pose2f(theLibCodeRelease.angleToGoal+30_deg, theBallModel.estimate.position.x() , theBallModel.estimate.position.y() - 85.f));
    }
  }
  
 state(dajiao)//大脚
  {
    transition
    {
      if(state_time > 3000 || (state_time > 10 && action_done))
        goto start;
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
	  SpecialAction(SpecialActionRequest::kickfoot);
     
    }
  }
  
   state(walkToRightTarget_)
  {
      transition
    {
          if(state_time > 3000)
           goto   turnToBall;
      if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
        goto searchForBall;
     
    }
    action
    {
    HeadControlMode(HeadControl::lookForward);
    WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(0_deg,0.f,-500.f ));
    }
    
   }
  
  state(walkToLeftTarget)
  {
      transition
    {
          if(state_time > 3000)
           goto   turnToBall;
      if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
        goto searchForBall;
    }
    action
    {
    HeadControlMode(HeadControl::lookForward);
    WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(0_deg,0.f,500.f ));
    }
    
   }
  
  
  
 }

