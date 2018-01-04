hasValue('bumperW', W) :- perceptionNorth([['bumperW', W], _, _]).
hasValue('bumperN', N) :- perceptionNorth([_, ['bumperN', N], _]).
hasValue('bumperE', E) :- perceptionNorth([_, _, ['bumperE', E]]).

takeDecision('North') :-
    hasValue('bumperN', 'False'), !.

takeDecision('West') :-
    hasValue('bumperW', 'False'), !.

takeDecision('Est') :-
    hasValue8('bumperE', 'False'), !.
  
takeDecision('South').
