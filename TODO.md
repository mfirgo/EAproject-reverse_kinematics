## Algorytmy ewolucyjne projekt: reverse kinematics

- napisać email na www

TODO:
done: log_dict
done: funkcja tworząca log dict i appendująca do history
done: measure time
done: funkcja tworząca frames z history
done: animacja obstacles
done: animacja arm
done: testing class
done: change color if collides

sample problems:
- done: single static
- done: double moving

algoritm info - wypisywanie/pokazywanie na animacji
done: cleanup algorytmu

przystosowanie algorytmu:
- ustalić procent niepoprawnej populacji
- done: zwiększyć sigmy niepoprawnych osobników - działa dla test2
- dodać losowych osobników przy zmianie czasu
- zmienić liczbę iteracji w zależności od czasu


TODO:
- Done: wykres objective value 
- Done: zwolnić animację
- done: zmienić funckcję celu na min
- done: zapisywać prawdziwą i zmodyfikowaną funkcję celu
- done: pokazywać prawdziwą funkcję celu

- done: iteration/time - ustawianie ilości czasu
- done: time_step

- napisac raport:
  * opis rozpatrywanego zagadnienia
    * DONE: robot arm
    * DONE: obstacle
    * DONE: target
  * dokładną definicję rozważanego problemu optymalizacji (przestrzeń poszukiwań,funkcja celu, itp.)
    * DONE: funkcja celu
    * DONE: ograniczenia
  * szczegółowy opis użytych algorytmów ewolucyjnych (jeśli nie są to klasyczne algorytmy omówione na wykładzie),
    * algorytm ES - 
      * DONE: liczba iteracji na dany czas, 
      * DONE: korzystanie z poprzedniej populacji 
      * DONE: modyfikacja funcji celu
  * szczegółowy opis implementacji użytych metod (jeśli nie jest ona oczywista)
  * szczegółowy opis uzyskanych wyników
  * wnioski końcowe, podsumowanie, 
  * perspektywy rozwoju:
    * zmiana algorytmu w taki sposób, żeby zamiast modyfikować funkcje celu przeżywało zawsze tylko pewien procent kolidujących z przeszkodami osobników, w razie niewystarczającej liczby poprawnych osobników można dodać losowe osobniki lub utworzyć więcej dzieci mutując tylko poprawne osobniki
    * poruszanie punktu celu
    * dla trudniejszych testów - typu labiryntu, może okazać się dobrym pomysłem zwiększenie maksymalnej liczby operacji dla czasu 0

