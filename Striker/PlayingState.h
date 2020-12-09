option(PlayingState)
{
  initial_state(demo)
  {
      transition
      {
          if((theGameInfo.kickingTeam != theOwnTeamInfo.teamNumber)&&flagg==0)
          {
              if(state_time>12000)
              {
                  flagg=1;
                  goto Striker;
              }
                  
          }
          
        else
              goto Striker;
      }
    action
    {
      LookRound();
      Stand();
      //Ceshi();
      //Defender();
      //SpecialActionDebug();
    }
  }
  
  state(Striker)
  {
      action
      {
             Strikertwo();
        }
      }
}
