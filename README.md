# Optymalizator Tras dla Dronów

Projekt z geometrii obliczeniowej — system planowania bezpiecznych tras lotu dla dronów w środowisku miejskim, z uwzględnieniem przeszkód (budynki) i stref zakazanych.

## Spis treści

- [Opis projektu](#opis-projektu)
- [Funkcjonalności](#funkcjonalności)
- [Wykorzystane algorytmy](#wykorzystane-algorytmy)
- [Struktura projektu](#struktura-projektu)
- [Instalacja](#instalacja)
- [Uruchomienie](#uruchomienie)

---

## Opis projektu

System wyznacza optymalną trasę dla drona pomiędzy dwoma punktami na mapie, omijając budynki i strefy zakazane. Wykorzystuje techniki z geometrii obliczeniowej do budowy grafu nawigacyjnego, a następnie algorytm A\* do znalezienia najkrótszej bezpiecznej ścieżki. Zapewnia wizualizację mapy, trasy i animację lotu drona, a także wyszukiwanie bezpiecznych stref lądowania.

## Funkcjonalności

- **Generowanie map** z konfigurowalną liczbą budynków i stref zakazanych (z seedem dla powtarzalności)
- **Wyznaczanie optymalnej trasy** między dwoma punktami z marginesem bezpieczeństwa
- **Walidacja kolizji** — sprawdzanie czy trasa nie przecina przeszkód
- **Optymalizacja trasy** poprzez eliminację zbędnych punktów pośrednich
- **Wyszukiwanie stref lądowania** o zadanym rozmiarze
- **Wizualizacja** z korytarzem bezpieczeństwa, numeracją punktów, długościami segmentów
- **Animacja lotu drona** po wyznaczonej trasie
- **Test wydajności** — pomiar czasu obliczeń dla losowych tras

## Wykorzystane algorytmy

### Triangulacja Delaunay
Dzieli przestrzeń mapy na trójkąty, których wierzchołki to punkty charakterystyczne przeszkód. Centroidy i wierzchołki trójkątów stają się węzłami grafu nawigacyjnego.

### Algorytm A\*
Heurystyczny algorytm wyszukiwania najkrótszej ścieżki w grafie. Wykorzystuje odległość euklidesową jako heurystykę.

### Drzewo zakresowe 2D
Struktura danych do efektywnego wyszukiwania punktów w zadanym zakresie współrzędnych — wykorzystywana przy optymalizacji wyszukiwania sąsiadów.

### Otoczka wypukła
Wyznaczanie otoczki wypukłej zbioru punktów — pomocnicze przy operacjach na wielokątach reprezentujących przeszkody.

### Operacje geometryczne (Shapely)
- Bufor (`buffer`) — powiększanie przeszkód o margines bezpieczeństwa
- Przecięcie (`intersects`) — sprawdzanie kolizji ścieżki z przeszkodami
- Korytarz bezpieczeństwa wokół trasy

## Struktura projektu

```
Optymalizator-trasy-dronow/
├── main.py                    # punkt wejścia — tryb demo i interaktywny
├── requirements.txt           # zależności Pythona
└── src/
    ├── geometry/              # podstawowe struktury geometryczne
    │   ├── point.py           # klasa Point (2D)
    │   ├── polygon.py         # wielokąty
    │   ├── convex_hull.py     # otoczka wypukła
    │   └── triangulation.py   # triangulacja Delaunay
    ├── map/                   # generowanie i reprezentacja mapy
    │   ├── map_data.py        # struktura danych mapy
    │   └── map_generator.py   # generator map z budynkami i strefami
    ├── pathfinding/           # wyznaczanie tras
    │   ├── drone_pathfinder.py  # główny pathfinder z A*
    │   └── range_tree.py      # drzewo zakresowe 2D
    └── visualization/
        └── visualizer.py      # wizualizacja matplotlib + animacja
```

## Instalacja

Wymagany Python 3.10+ oraz `pip`.

```bash
git clone https://github.com/Alan2113/Optymalizator-trasy-dronow
cd Optymalizator-trasy-dronow/Optymalizator-trasy-dronow
python -m venv .venv
source .venv/bin/activate          # Linux/macOS
# .venv\Scripts\activate           # Windows
pip install -r requirements.txt
```

## Uruchomienie

### Tryb interaktywny (domyślny)

```bash
python main.py
```

Menu pozwala:
1. Wygenerować nową mapę (liczba budynków, stref, seed, margines bezpieczeństwa)
2. Wyznaczyć trasę między wybranymi punktami
3. Zwizualizować mapę
4. Wyszukać bezpieczne strefy lądowania
5. Uruchomić test wydajności

### Tryb demonstracyjny

```bash
python main.py --demo
```

Automatycznie generuje mapę, wyznacza trasę z rogu do rogu, wyświetla animację lotu i zapisuje wynik do pliku PNG.

