# MiniEFIS: opis urządzenia

## Czym jest MiniEFIS
MiniEFIS to mały elektroniczny przyrząd pokładowy montowany w tablicy przyrządów jak klasyczny wskaźnik. Ma okrągły, kolorowy ekran dotykowy i w jednej obudowie łączy kilka przyrządów: sztuczny horyzont, wysokościomierz z wariometrem, zakrętomierz z chyłomierzem, kompas i wskaźnik nawigacyjny HSI.

Urządzenie ma wbudowane czujniki położenia, kompas magnetyczny i barometr. Zaawansowane algorytmy łączą ich wskazania tak, że horyzont pozostaje stabilny także w zakrętach, a wysokość i prędkość pionowa reagują szybko i bez szumów.

Opcjonalnie można podłączyć zewnętrzne moduły:
- **GPS**, potrzebny do nawigacji po trasie (ekran HSI),
- **czujnik prędkości i temperatury zewnętrznej,**
- **moduł silnika** (obroty, temperatury, ciśnienia, paliwo).

W jednym samolocie może pracować kilka MiniEFIS korzystających z tych samych modułów.

## Obsługa
- **Podwójne stuknięcie** w ekran otwiera **menu** z listą wszystkich ekranów. Stuknięcie poza menu je zamyka.
- **Przesunięcie palcem w lewo lub w prawo** przełącza przyrządy po kolei: horyzont → wysokościomierz → zakrętomierz → kompas → HSI i znowu od początku. Ustawienia i diagnostyka są dostępne tylko z menu.
- Urządzenie **pamięta ostatni ekran** i wszystkie ustawienia, także po wyłączeniu zasilania.

---

## 1. Sztuczny horyzont
- **Tło:** niebo i ziemia, obracane zgodnie z przechyleniem i pochyleniem samolotu.
- **Drabinka pochylenia:** kreski co 10°.
- **Skala przechylenia:** łuk u góry ekranu.
- **Liczby:** aktualne przechylenie i pochylenie.
- **Panel z prawej strony:** wysokość i prędkość pionowa.
- **Stan przyrządu** na dole ekranu:
  - **AHRS** (biały): wszystko w porządku,
  - **GYRO ONLY** (bursztynowy): ograniczona dokładność,
  - **CAL – HOLD STILL**: przygotowanie po włączeniu; urządzenie musi wtedy pozostać nieruchome,
  - **NO IMU** (czerwony): awaria.

## 2. Wysokościomierz
- **Tarcza z dwiema wskazówkami:** długa pokazuje setki, krótka tysiące. Wysokość jest też wyświetlana liczbowo.
- **Wariometr:** pasek wzdłuż lewej krawędzi, zielony przy wznoszeniu i pomarańczowy przy opadaniu.
- **Ustawienie ciśnienia QNH:**
  - przyciski **−** i **+** zmieniają je o 1 hPa (przytrzymanie powtarza),
  - **STD** ustawia ciśnienie standardowe 1013 hPa.
- **Jednostki:** metry albo stopy.

## 3. Zakrętomierz
- **Sylwetka samolotu** przechyla się w zakręcie. Skrzydła na dolnych znacznikach oznaczają zakręt standardowy (pełny okrąg w 2 minuty). Prędkość zakrętu jest też wyświetlana liczbowo.
- **Kulka** pokazuje ślizg lub wyślizg. W zakręcie skoordynowanym kulka jest na środku.
- **Tłumienie:** wskazania są wygładzone jak w przyrządzie mechanicznym, więc nie reagują na drgania i turbulencję.

## 4. Kompas
- **Róża kompasowa** obraca się pod nieruchomą sylwetką samolotu.
- **Kurs magnetyczny** jest wyświetlany liczbowo.
- **Kalibracja kompasu** jest potrzebna jednorazowo, po zamontowaniu urządzenia w samolocie. Bez niej ekran nie pokazuje kursu, tylko informuje:
  - **NOT CALIBRATED**: wykonaj kalibrację w ustawieniach,
  - **CALIBRATING**: kalibracja w toku,
  - **NO HEADING**: zakłócenie magnetyczne,
  - **NO MAGNETOMETER**: awaria czujnika.

## 5. HSI (nawigacja po trasie)
- **Wymaga modułu GPS** i trasy wczytanej z pliku GPX przez przeglądarkę.
- **Róża kursowa:** u góry jest zawsze aktualny kierunek lotu.
- **Wskaźniki na róży:**
  - **magentowy trójkąt:** kierunek do aktywnego punktu trasy,
  - **zielony trójkąt:** zadany kurs odcinka.
- **Belki odchylenia** na środku. Lecimy w stronę belki.
  - **pionowa belka:** odchylenie w bok od linii trasy,
  - **pozioma belka:** odchylenie od zaplanowanej wysokości, jeśli trasa zawiera wysokości punktów.
- **Pola liczbowe:**
  - **WPT:** nazwa punktu,
  - **BRG:** kierunek do niego,
  - **DIST:** odległość,
  - **XTE:** odchylenie od trasy, z literą L/R (po której stronie trasy jest samolot).
- **Automatyczne przełączanie punktów:** po minięciu punktu przyrząd sam przechodzi do następnego.
- **Menu trasy:** **przesunięcie palcem z góry na dół** otwiera listę punktów trasy. Aktywny punkt jest podświetlony.
  - **stuknięcie w nazwę punktu:** lot prosto do niego z aktualnego miejsca („direct to”),
  - **LEG** obok nazwy: powrót na zaplanowany odcinek prowadzący do tego punktu.
- **Pamięć trasy:** trasa przetrwa wyłączenie zasilania.
- **Ostrzeżenia:** **NO ROUTE** (brak trasy) lub **NO GPS** (brak sygnału GPS).

## 6. Ustawienia
- **Jednostki:** metryczne albo lotnicze.
- **Jasność ekranu.**
- **„Set level”:** korekta, gdy urządzenie jest zamontowane krzywo w tablicy. Wykonuje się ją przy wypoziomowanym, nieruchomym samolocie.
- **Kalibracja kompasu:** prowadzi krok po kroku przez obracanie samolotu.
- **Wi-Fi:** włącza dostęp do urządzenia z telefonu lub komputera.

## 7. Diagnostyka
Odczyty wszystkich czujników, przydatne przy instalacji i szukaniu usterek.

---

## Dostęp przez Wi-Fi
Po włączeniu Wi-Fi w przeglądarce telefonu lub komputera można:
- **wczytać trasę z pliku GPX**, przejrzeć jej punkty i wybrać „direct to” do dowolnego z nich,
- **oglądać wykresy** wskazań na żywo,
- **sprawdzić podłączone moduły** (GPS, silnik itd.),
- **zaktualizować oprogramowanie** bezprzewodowo; nieudana aktualizacja automatycznie wraca do poprzedniej wersji.
