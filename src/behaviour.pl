hasValue('bumperW', W) :- perceptionBumper([['bumperW', W], _, _]).
hasValue('bumperN', N) :- perceptionBumper([_, ['bumperN', N], _]).
hasValue('bumperE', E) :- perceptionBumper([_, _, ['bumperE', E]]).

takeDecision('TurnSouth') :-
    hasValue('bumperN', 'True'), hasValue('bumperW', 'True'), hasValue('bumperE', 'True'), !.

takeDecision('GoStraight') :-
    hasValue('bumperN', 'False'), hasValue('bumperW', 'False'), hasValue('bumperE', 'False') !.

takeDecision('TurnWest') :-
    hasValue('bumperW', 'False'), !.

takeDecision('TurnEast') :-
    hasValue('bumperE', 'False').
  
