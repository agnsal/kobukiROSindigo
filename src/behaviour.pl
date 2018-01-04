hasValue('bumperW', W) :- perceptionBumper([['bumperW', W], _, _]).
hasValue('bumperN', N) :- perceptionBumper([_, ['bumperN', N], _]).
hasValue('bumperE', E) :- perceptionBumper([_, _, ['bumperE', E]]).

takeDecision('GoStraight') :-
    hasValue('bumperN', 'False'), !.

takeDecision('TurnWest') :-
    hasValue('bumperW', 'False'), !.

takeDecision('TurnEst') :-
    hasValue8('bumperE', 'False'), !.
  
takeDecision('TurnSouth').
