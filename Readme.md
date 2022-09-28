# Algorytmy ewolucyjne projekt: reverse kinematics

Projekt z przedmiotu algorytmy ewolucyjne 2021/2022 na Uniwersytecie Wrocławskim.  
Autor: Martyna Firgolska  

Raport z projektu znajduje się w pliku test_cases.ipynb  

Projekt wykorzystuje bibliotekę shapely https://shapely.readthedocs.io/en/stable/manual.html  
```pip3 install shapely```

### Klasy użyte w projekcie:

#### Algoritm - plik algorytm.py

#### Obstacle, SimpleObstacle - plik obstacle.py

SimpleObstacle korzysta z bibioteki shapely
SimpleObstacle.current_obj i .starting_obj są obiektami typu shapely.LinearRing ale mogą być dowolnymi obiektami geometrycznymi
Simple Obstacle używa funkcji intersects z biblioteki shapely aby wykrywać kolizje

#### RobotArm - plik robot_arm.py
#### ReverseKinProblem - plik reverse_kin_problem.py
Do znajdowania kolidujących indeksów zamieniamy osobniki na objekty LineString i korzystamy z Obstacle.intersects które korzysta z shapely

#### EvolutionAnimation - plik evolution_animation.py

#### AlgoritmTest - plik test_examples.py 
W tym pliku znajdują się również definicje testów użytych w raporcie i kilka funkcji ułatwiających tworzenie testów.
