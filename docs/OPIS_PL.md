# MiniEFIS: opis urządzenia

## Czym jest MiniEFIS
MiniEFIS to mały elektroniczny przyrząd pokładowy montowany w tablicy przyrządów jak klasyczny wskaźnik. Ma okrągły, kolorowy ekran dotykowy i w jednej obudowie łączy kilka przyrządów: sztuczny horyzont, wysokościomierz z wariometrem, zakrętomierz z chyłomierzem, kompas, wskaźnik nawigacyjny HSI oraz wskaźniki silnika.

Urządzenie ma wbudowane czujniki położenia, kompas magnetyczny i barometr. Zaawansowane algorytmy łączą ich wskazania tak, że horyzont pozostaje stabilny także w zakrętach, a wysokość i prędkość pionowa reagują szybko i bez szumów.

Opcjonalnie można podłączyć zewnętrzne moduły:
- **GPS**, potrzebny do nawigacji po trasie (ekran HSI),
- **czujnik prędkości i temperatury zewnętrznej,**
- **moduł silnika** (obroty, temperatury, ciśnienia, paliwo), potrzebny do ekranów silnika i cylindrów.

W jednym samolocie może pracować kilka MiniEFIS korzystających z tych samych modułów.

## Obsługa
- **Podwójne stuknięcie** w ekran otwiera **menu** z listą wszystkich ekranów. Stuknięcie poza menu je zamyka.
- **Przesunięcie palcem w lewo lub w prawo** przełącza przyrządy po kolei: horyzont → wysokościomierz → zakrętomierz → kompas → HSI → silnik → cylindry i znowu od początku. Ustawienia są dostępne tylko z menu.
- **Niepotrzebne ekrany można wyłączyć** w ustawieniach, np. HSI, gdy nie ma modułu GPS. Wyłączony ekran znika z menu i jest pomijany przy przesuwaniu.
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

## 6. Silnik
- **Wymaga modułu silnika.**
- **Obrotomierz** na górze ekranu: łuk z zakresami zielonym, żółtym i czerwonym, wskazówka i obroty liczbowo. Pod nimi ciśnienie w kolektorze ssącym, jeśli silnik je mierzy.
- **Paski** z zakresami i znacznikiem wartości:
  - ciśnienie i temperatura oleju,
  - temperatura najgorętszej głowicy (CHT),
  - temperatura płynu chłodzącego (Rotax) albo najgorętszych spalin (EGT, Lycoming),
  - ciśnienie paliwa, przepływ paliwa i ilość paliwa w zbiornikach,
  - napięcie instalacji.
- **Kolory liczb:** biała w normie, żółta lub czerwona poza nią.

## 7. Cylindry
- **Słupki EGT i CHT** dla każdego cylindra obok siebie, z czerwoną linią granicy CHT. Pod nimi wartości liczbowo; numer najgorętszego cylindra jest wyróżniony.
- **Na górze:** temperatura powietrza w gaźniku (oblodzenie) i prąd ładowania akumulatora.
- **Na dole:** zapas czasu lotu na paliwie, zużyte paliwo i czas pracy silnika.

Na obu ekranach brak danych z modułu jest pokazany kreskami, a gdy moduł milczy całkowicie: **NO ENGINE DATA**.

**Menu ekranów silnika:** przesunięcie palcem z góry na dół na ekranie silnika lub cylindrów otwiera ustawienia tego, co jest pokazywane:
- **słupki EGT i CHT** na ekranie cylindrów (co najmniej jeden rodzaj zostaje),
- **MAP** pod obrotomierzem,
- **wartości na paskach** ekranu silnika, do ośmiu: ciśnienie i temperatura oleju, najgorętsze CHT i EGT, płyn chłodzący, ciśnienie, przepływ i ilość paliwa, napięcie, prąd akumulatora, temperatura powietrza w gaźniku.

Liczba cylindrów wynika z typu silnika (dla silnika własnego ustawia się ją w ustawieniach). Zmiana typu silnika przywraca jego domyślny zestaw wartości.

## 8. Ustawienia
Ustawienia to lista działów. Każdy otwiera się na osobnej stronie z przyciskiem **Back** (powrót):
- **Display:** jednostki (metryczne albo lotnicze) i jasność ekranu.
- **Screens:** włączanie i wyłączanie poszczególnych przyrządów. Co najmniej jeden zostaje włączony.
- **Engine:** typ silnika – Rotax 912, Lycoming O-320, IO-540 albo **własny (Custom)**. Od typu zależą zakresy kolorów na ekranach silnika; warto je sprawdzić z instrukcją silnika.
  - Dla silnika własnego **Edit custom engine** otwiera listę wartości (obroty, ciśnienie w kolektorze, ciśnienie i temperatura oleju, CHT, płyn chłodzący, EGT, ciśnienie, przepływ i ilość paliwa, napięcie, prąd akumulatora, powietrze w gaźniku). Dla każdej ustawia się przyciskami **−** i **+** zakres skali oraz granice zakresów żółtego i czerwonego, od dołu i od góry. Stuknięcie w granicę ją wyłącza albo włącza.
  - Osobno: liczba cylindrów i czy silnik jest chłodzony cieczą.
  - Silnik własny można zacząć od kopii jednego z gotowych typów.
- **Mounting:** „Set level” – korekta, gdy urządzenie jest zamontowane krzywo w tablicy. Wykonuje się ją przy wypoziomowanym, nieruchomym samolocie.
- **Wi-Fi:** włącza dostęp do urządzenia z telefonu lub komputera.
- **Compass:** odczyt magnetometru na żywo i kalibracja kompasu, która prowadzi krok po kroku przez obracanie samolotu.

---

## Dostęp przez Wi-Fi
Po włączeniu Wi-Fi w przeglądarce telefonu lub komputera można:
- **wczytać trasę z pliku GPX**, przejrzeć jej punkty i wybrać „direct to” do dowolnego z nich,
- **oglądać wykresy** wskazań na żywo,
- **sprawdzić podłączone moduły** (GPS, silnik itd.),
- **zaktualizować oprogramowanie** bezprzewodowo; nieudana aktualizacja automatycznie wraca do poprzedniej wersji.
