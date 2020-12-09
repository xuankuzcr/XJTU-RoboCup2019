option(Strikertwo)
{
  std::vector<Obstacle> o = theObstacleModel.obstacles;  
  std::vector<Teammate> t = theTeamData.teammates;
  static float pianLiDiany; 
  static float pianLiDianx;
  static float flag = 1.f;
  static float ballflag=1.f;//1代表自己看到球，0使用队友的球坐标
  static Vector2f realBall;
  const  Vector2f pianLiDian    = Transformation::fieldToRobot(theRobotPose,Vector2f(pianLiDianx,pianLiDiany)); 
  const  Vector2f midSpot  = Transformation::fieldToRobot(theRobotPose,Vector2f(2000.f,0));
  const  Vector2f leftSpot      = Transformation::fieldToRobot(theRobotPose,Vector2f(2500.f,1000.f));
  const  Vector2f rightSpot       = Transformation::fieldToRobot(theRobotPose,Vector2f(2500.f,-1000.f));
  const  Vector2f zhongDian     = Transformation::fieldToRobot(theRobotPose,Vector2f(0,0));
  const  Vector2f ball          = theBallModel.estimate.position;
  const  Vector2f globalBall    = Transformation::robotToField(theRobotPose,theBallModel.estimate.position);
  const  Vector2f globalRobot   = theRobotPose.translation;
  initial_state(start)
  {
    transition
    {
      if(state_time > 1000.f||(theLibCodeRelease.timeSinceBallWasSeen < 300&&state_time>1000.f))
      {
        goto turnToBall;
      }
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
      //Stand();
      WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(0_deg,2000.f,0));
    }
  }
  
    state(restart)
  {
    transition
    {
      if(state_time > 4000.f||(theLibCodeRelease.timeSinceBallWasSeen < 300&&state_time>2000))
      {
        goto turnToBall;
      }
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
      //Stand();
      WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(0_deg,1000.f,0));
    }
  }
  
  state(turnToBall)
  {
    transition
    {
        
      if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
        goto searchForBall;
      if(std::abs(theBallModel.estimate.position.angle()) < 3_deg)
        goto walkToBall;
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
      WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(theBallModel.estimate.position.angle(), 0.f, 0.f));
    }
  }

  state(walkToBall)
  {
    transition
    {
       if(theLibCodeRelease.timeSinceBallWasSeen > 4000.f)
      //if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
        goto searchForBall;
        
         size_t i=0; 
         for( i=0;i<o.size();i++)
        {
           if(o[i].center.x()>0&&o[i].center.x()<300
           &&std::abs(o[i].center.y())<200
           &&(o[i].type==Obstacle::someRobot||o[i].type==Obstacle::opponent)
           &&theBallModel.estimate.position.x()>o[i].center.x()+50.f)
           {

             if(o[i].center.y()>=0)
                 goto walkToRightTargets;
             else
                 goto walkToLeftTargets; 
           }
        }
        
        
		if(theBallModel.estimate.position.norm() < 300.f&&(globalRobot.x()-globalBall.x())<-50)
          goto alignToGoal;
          
          
		if(theBallModel.estimate.position.norm() < 300.f&&(globalRobot.x()-globalBall.x())>-100)
        {
            if(globalBall.y()>=0)
                goto raoQiuRight;
            else if(globalBall.y()<0)
                goto raoQiuLeft;
        }
          
          
          
      if(state_time>2000&&std::abs(theBallModel.estimate.position.angle()) > 40_deg)
        goto turnToBall;  
        
    }
    action
    {
      //HeadControlMode(HeadControl::lookForward);
      LookAtBall();
      WalkToTarget(Pose2f(50.f, 50.f, 50.f), theBallModel.estimate.position);
    }
  }

  state(alignToGoal)
  {
    transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen > 7000.f)
     // if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
        goto searchForBall;

  

      if(std::abs(theLibCodeRelease.angleToGoal) < 10_deg && std::abs(theBallModel.estimate.position.y()) < 100.f)
         goto alignBehindBall;

    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
      WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(theLibCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 350.f, theBallModel.estimate.position.y()));
    }
  }

  state(raoQiuRight)
  {
    transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
        goto searchForBall;
      if(std::abs(theLibCodeRelease.angleToGoal) < 15_deg)
        goto turnToBall;
        //goto alignBehindBall;
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
	  WalkToTarget(Pose2f(80.f, 80.f, 80.f), Pose2f(ball.angle(), ball.x() - 240.f, -100.f));
    }
  }
  
   state(raoQiuLeft)
  {
    transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
        goto searchForBall;
      if(std::abs(theLibCodeRelease.angleToGoal) < 15_deg)
        goto turnToBall;
        //goto alignBehindBall;
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
	  WalkToTarget(Pose2f(80.f, 80.f, 80.f), Pose2f(ball.angle(), ball.x() - 240.f, 100.f));  
    }
  }
  
  state(alignBehindBall)
  {
    transition
    {
        
   if(theLibCodeRelease.timeSinceBallWasSeen > 4000)
        goto searchForBall;
    size_t i=0;
      for( i=0;i<o.size();i++)
        {
           if(o[i].center.x()>50.f&&o[i].center.x()<800.f&&std::abs(o[i].center.y())<150
           &&(o[i].type==Obstacle::teammate||o[i].type==Obstacle::opponent)
           &&zhongDian.norm()<2000.f)
           {

             if(globalRobot.y()>=-200.f)
                 goto alignSideKickLeft;
             if(globalRobot.y()<=200.f)
                 goto alignSideKickRight; 
           }
        }
      for( i=0;i<o.size();i++)
        {
           if(o[i].center.x()>0&&o[i].center.x()<4000.f&&std::abs(o[i].center.y())<300.f
           &&(o[i].type==Obstacle::teammate||o[i].type==Obstacle::opponent))
           {
                 if(o[i].left.y()>-100.f&&o[i].right.y()<100.f)
                 {
                 Vector2f globalOb = Transformation::robotToField(theRobotPose,o[i].center);
                 if(globalRobot.x()>2000.f||o[i].center.x()>2000.f)
                 {
                  if(globalOb.y()<=-200.f&&globalBall.y()<=0)
                  {
                      pianLiDiany=330.f;
                      pianLiDianx=4700.f;
                      goto alignToPianLiDian;
                  }
                  if(globalOb.y()>=-200.f&&globalBall.y()<=0)
                  {
                      pianLiDiany=-380.f;
                      pianLiDianx=4700.f;
                      goto alignToPianLiDian;
                  }    
                  if(globalOb.y()<=200.f&&globalBall.y()>=0)
                  {
                      pianLiDiany=380.f;
                      pianLiDianx=4700.f;
                      goto alignToPianLiDian;
                  }
                  if(globalOb.y()>=200.f&&globalBall.y()>=0)
                  {
                      pianLiDiany=-330.f;
                      pianLiDianx=4700.f;
                      goto alignToPianLiDian;
                  }  
 
                  if(o[i].center.y()>0) 
                  {
                      pianLiDiany=globalRobot.y()+o[i].right.y()-600.f+std::abs(0.02*globalRobot.x())+std::abs(0.1*globalRobot.y());
                  }    
                  else
                      pianLiDiany=globalRobot.y()+o[i].left.y()+600.f-std::abs(0.02*globalRobot.x())-std::abs(0.1*globalRobot.y());
                    
                      pianLiDianx=4700.f;
                      goto alignToPianLiDian;
                 }
           }
        }
        }
   
      if(theLibCodeRelease.between(theBallModel.estimate.position.y(), 35.f, 55.f)
         && theLibCodeRelease.between(theBallModel.estimate.position.x(), 150.f, 175)
         && std::abs(theLibCodeRelease.angleToGoal) < 2_deg)
         {

             if(   std::abs(globalRobot.x())<1500||  globalRobot.x()>3800 )
                 goto kick;
            else
                goto daJiao;
         }
               


         
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      WalkToTarget(Pose2f(20.f, 20.f, 20.f), Pose2f(theLibCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 165.f, theBallModel.estimate.position.y() - 42.f));
    }
  }

  state(searchForBall)
  {
    transition
    {
        
      if(theLibCodeRelease.timeSinceBallWasSeen <200)
      {
          ballflag=1.f;
          goto turnToBall;
      }
   /* for(int i=0;i<t.size();i++)
    {
      if(theLibCodeRelease.timeSinceBallWasSeen < 300)
      {
          ballflag=1.f;
          goto turnToBall;
      }
         if(theFrameInfo.getTimeSince(t[i].ball.timeWhenLastSeen)<200*2/3) 
        {    
            ballflag=0;
            goto turnToball;
        }
                
    }*/
        
      if(state_time>8000)
      {
        if(flag==1.f)
            goto turnToMidSpot;
        if(flag==2.f)
            goto turnToRightSpot;
        if(flag==3.f)
            goto turnToLeftSpot;
      }
    }
    action
    {
      if(state_time<2600)
      {
          Stand();
          LookRound();
      } 
      else
      { 
      HeadControlMode(HeadControl::lookForward);
      WalkAtRelativeSpeed(Pose2f(1.f, 0.f, 0.f));
      }
      }
  }
  
 state(searchForBallLeft)
  {
    transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen < 300&&state_time>1600.f)
        goto turnToBall;
      if(state_time>3000)
        goto turnToMidSpot;
    }
    action
    {
      if(state_time<1600)
      {
          Stand();
          LookRight();
      } 
      else
      { 
      HeadControlMode(HeadControl::lookForward);
      WalkAtRelativeSpeed(Pose2f(-1.f, 0.f, 0.f));
      }
      }
    }
  
  state(searchForBallRight)
  {
    transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen < 300&&state_time>1600.f)
        goto turnToBall;
      if(state_time>3000)
        goto turnToMidSpot;
    }
    action
    {
      if(state_time<1600)
      {
          Stand();
          LookLeft();
      } 
      else
      { 
      HeadControlMode(HeadControl::lookForward);
      WalkAtRelativeSpeed(Pose2f(1.f, 0.f, 0.f));
      }
      }
    }
    
  state(turnToMidSpot)
  {
    transition
    {
      if(midSpot.norm()<200.f)
        goto searchForBall;
      if(theLibCodeRelease.timeSinceBallWasSeen < 300)
      {
        flag=1.f;  
        goto turnToBall;
      }
      if(std::abs(midSpot.angle()) < 3_deg)
        goto yiDongToMidSpot;
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
      WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(midSpot.angle(), 0.f, 0.f));
    }
  }
  
  state(yiDongToMidSpot)
  {
     transition 
     {
        if(theLibCodeRelease.timeSinceBallWasSeen < 200)
        {
          flag=1.f;
          goto turnToBall; 
        }
        if(midSpot.norm()<200.f)
        {
            flag=2.f;
            goto searchForBall;
        }
        if(state_time>14000.f)
        goto searchForBall;
        if(state_time>2000&&std::abs(midSpot.angle()) > 40_deg)
        goto turnToMidSpot;  
     }
     action
     {
         LookRoundLow();
         //HeadControlMode(HeadControl::lookForward);
         WalkToTarget(Pose2f(50.f,50.f,50.f),Pose2f(0_deg,midSpot.x(),midSpot.y()));
     }
  }
  
   state(turnToRightSpot)
  {
    transition
    {
      if(rightSpot.norm()<200.f)
        goto searchForBall;
      if(theLibCodeRelease.timeSinceBallWasSeen < 300)
      {
        flag=1.f;  
        goto turnToBall;
      }
      if(std::abs(rightSpot.angle()) < 2_deg)
        goto yiDongToRightSpot;
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
      WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(rightSpot.angle(), 0.f, 0.f));
    }
  }
  
  state(yiDongToRightSpot)
  {
     transition 
     {
        if(theLibCodeRelease.timeSinceBallWasSeen < 200)
        {
          flag=1.f;
          goto turnToBall; 
        }
        if(rightSpot.norm()<200.f)
        {
            flag=3.f;
            goto searchForBall;
        }
        if(state_time>14000.f)
        goto searchForBall;
      if(state_time>2000&&std::abs(rightSpot.angle()) > 40_deg)
        goto turnToRightSpot;  
     }
     action
     {
         LookRoundLow();
         // HeadControlMode(HeadControl::lookForward);
         WalkToTarget(Pose2f(50.f,50.f,50.f),Pose2f(0_deg,rightSpot.x(),rightSpot.y()));
     }
  }
  
   state(turnToLeftSpot)
  {
    transition
    {
      if(leftSpot.norm()<200.f)
        goto searchForBall;
      if(theLibCodeRelease.timeSinceBallWasSeen < 300)
      {
        flag=1.f;  
        goto turnToBall;
      }
      if(std::abs(leftSpot.angle()) < 2_deg)
        goto yiDongToLeftSpot;
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
      WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(leftSpot.angle(), 0.f, 0.f));
    }
  }
  
  state(yiDongToLeftSpot)
  {
     transition 
     {
        if(theLibCodeRelease.timeSinceBallWasSeen < 200)
        {
          flag=1.f;
          goto turnToBall; 
        }
        if(leftSpot.norm()<200.f)
        {
            flag=1.f;
            goto searchForBall;
        }
        if(state_time>14000.f)
        goto searchForBall;
                if(state_time>2000&&std::abs(leftSpot.angle()) > 40_deg)
        goto turnToLeftSpot;  
     }
     action
     {
        LookRoundLow();
      //   HeadControlMode(HeadControl::lookForward);
         WalkToTarget(Pose2f(50.f,50.f,50.f),Pose2f(0_deg,leftSpot.x(),leftSpot.y()));
     }
  }
  
  state(alignSideKickLeft)
  {
    transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen > 3000.f)
        goto searchForBall;
      if(theLibCodeRelease.between(theBallModel.estimate.position.y(), -30.f, 0.f)
         && theLibCodeRelease.between(theBallModel.estimate.position.x(), 160.f, 190.f))
        goto sideKickLeft;
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      WalkToTarget(Pose2f(20.f, 20.f, 20.f), Pose2f(theLibCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 180.f, theBallModel.estimate.position.y() + 30.f));
    }
  }
  
  state(sideKickLeft)
  {
    transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen > 3000.f)
        goto searchForBall;
      if(state_time > 3000 || (state_time > 10 && action_done))
        goto searchForBallLeft;
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
      InWalkKick(WalkKickVariant(WalkKicks::sidewardsInner, Legs::left), Pose2f(theLibCodeRelease.angleToGoal, theBallModel.estimate.position.x()-100.f , theBallModel.estimate.position.y() - 80.f));
    }
  }
  
  state(alignSideKickRight)
  {
    transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen > 2000.f)
        goto searchForBall;
      if(theLibCodeRelease.between(theBallModel.estimate.position.y(), 0.f, 30.f)
         && theLibCodeRelease.between(theBallModel.estimate.position.x(), 160.f, 190.f))
        goto sideKickRight;
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      WalkToTarget(Pose2f(20.f, 20.f, 20.f), Pose2f(theLibCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 180.f, theBallModel.estimate.position.y() - 30.f));
    }
  }

  state(sideKickRight)
  {
    transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen > 3000.f)
        goto searchForBall;
      if(state_time > 3000 || (state_time > 10 && action_done))
        goto searchForBallRight;
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
      InWalkKick(WalkKickVariant(WalkKicks::sidewardsInner, Legs::right), Pose2f(theLibCodeRelease.angleToGoal, theBallModel.estimate.position.x()-10.f , theBallModel.estimate.position.y() + 80.f));
    }
  }
  
  state(walkToRightTargets)
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
  
  state(walkToLeftTargets)
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

  state(alignToPianLiDian)
  {
    transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen > 4000.f)
        goto searchForBall;
      if(std::abs(pianLiDian.angle()) < 10_deg && std::abs(theBallModel.estimate.position.y()) < 100.f)
        goto alignBehindBallToPianLiDian;
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
      WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(pianLiDian.angle(), theBallModel.estimate.position.x() - 250.f,     theBallModel.estimate.position.y()));
    }
  }
   
  state(alignBehindBallToPianLiDian)
  {
    transition
    {

      if(theLibCodeRelease.timeSinceBallWasSeen > 5000.f)
        goto searchForBall;
        
      if(theLibCodeRelease.between(theBallModel.estimate.position.y(), 40.f, 70.f)
         && theLibCodeRelease.between(theBallModel.estimate.position.x(), 140.f, 170.f)
         && std::abs(pianLiDian.angle()) < 2_deg)
         {
       // if(globalRobot.x()<3500)
            goto daJiao;
       /* else
            goto kick;*/
         }
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      WalkToTarget(Pose2f(20.f, 20.f, 20.f), Pose2f(pianLiDian.angle(), theBallModel.estimate.position.x() - 165.f,        theBallModel.estimate.position.y() - 50.f));
    }
  }
   
  state(kick)
  {
    transition
    {
      if(state_time > 4000 || (state_time > 10 && action_done))
        goto  restart;
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
      InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(theLibCodeRelease.angleToGoal, theBallModel.estimate.position.x() -160.f, theBallModel.estimate.position.y() - 55.f));
    }
  }
  
   state(daJiao)
  {
    transition
    {
      if(state_time > 4000 || (state_time > 10 && action_done))
        goto   restart;
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
        SpecialAction(SpecialActionRequest::dajiao);    
    }
  } 

}
