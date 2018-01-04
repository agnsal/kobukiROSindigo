hasValue('bumperW', W) :- perceptionBumper([['bumperW', W], _, _]).
hasValue('bumperN', N) :- perceptionBumper([_, ['bumperN', N], _]).
hasValue('bumperE', E) :- perceptionBumper([_, _, ['bumperE', E]]).

takeDecision('TurnSouth') :-
    hasValue('bumperN', '1'), hasValue('bumperW', '1'), hasValue('bumperE', '1'), !.

takeDecision('GoStraight') :-
    hasValue('bumperN', '0'), !.

takeDecision('TurnWest') :-
    hasValue('bumperW', '0'), !.

takeDecision('TurnEast') :-
    hasValue('bumperE', '0').
  
