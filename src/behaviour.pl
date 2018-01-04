hasValue('bumperW', W) :- perceptionNorth([['bumperW', W], _, _]).
hasValue('bumperN', N) :- perceptionNorth([_, ['bumperN', N], _]).
hasValue('bumperE', E) :- perceptionNorth([_, _, ['bumperE', E]]).

takeDecision('GoStraight') :-
    hasValue('bumperN', 'False'), !.

takeDecision('TurnWest') :-
    hasValue('bumperW', 'False'), !.

takeDecision('TurnEst') :-
    hasValue8('bumperE', 'False'), !.
  
takeDecision('TurnSouth').
