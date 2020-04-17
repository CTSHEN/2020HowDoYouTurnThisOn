How to run:
====================
python3 main.py -control 0 -plan 0
python3 main.py -control 0 -plan 1
python3 main.py -control 1 -plan 0
python3 main.py -control 1 -plan 1

Control Type
====================
0 - Pure Pursuit
1 - Stanley

Plan Type
====================
0 - Astar
1 - RRT Star

BONUS
====================
If collision, car will reverse (straight line) then redirect to the end point.