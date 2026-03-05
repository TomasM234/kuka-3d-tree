# Specifikace Konverze: G-Code -> Univerzální Robotická Struktura -> CSV

Tento dokument definuje proces transformace standardního G-Code pro 3D tisk (vygenerovaného např. PrusaSlicerem) do bezstavové, deterministické datové struktury určené pro programování průmyslových 3D tiskových robotů (např. KUKA, ABB) a následný export do paměťově úsporného CSV formátu.

## 1. Vstupní Formát (G-Code)
G-Code je sekvenční textový formát, který je silně **stavový**. Každý řádek obsahuje pouze změnové příkazy; hodnoty, které nejsou uvedeny, zůstávají stejné jako na předchozím řádku.

### Zpracovávané parametry:
*   **Geometrie:** Souřadnice `X`, `Y` a `Z` [mm]. Jsou vždy zpracovávány jako absolutní pozice (`G90`).
*   **Extruze:** Souřadnice `E` [mm]. Představuje absolutní množství posunutého filamentu. Slicer často provádí nulování pomocí `G92 E0` jako obranu proti ztrátě přesnosti (floating point precision).
*   **Rychlost:** Parametr `F` [mm/min] aplikovaný na přesun příkazem `G1`.
*   **Stavy (M-kódy):** Teplota hotendu (`M104`, `M109`), chladící ventilátor (`M106`, `M107`), a průběh tisku (`M73`).

### Cíl parseru:
Parser musí sekvenčně číst řádky, udržovat si interní paměťový stav naprosto všech os a proměnných, a pro každý interpretovaný pohybový příkaz (`G1`) vygenerovat plně deterministický matematický vektor (viz sekce 2).

---

## 2. Interní Paměťová Struktura (Univerzální Datový Model)
Pro účely pokročilých výpočtů (blending, TCP kontrola), vizualizací a následného post-processingu pro různé jazyky robotů (KRL, RAPID) udržuje konvertor v RAM (nebo bufferu) plnou reprezentaci každého pohybu.

Každý vygenerovaný "bod" (Element) v poli musí obsahovat následující informace:

```json
{
  "type": "PRINT",               // Výčet chování: "PRINT", "TRAVEL", "RETRACT", "UNRETRACT"
  
  "position": {
    "x": 327.041,                // [mm] Absolutní koncová poloha X pro tento segment
    "y": 369.423,                // [mm] Absolutní koncová poloha Y pro tento segment
    "z": 1.000                   // [mm] Absolutní koncová poloha Z pro tento segment
  },
  
  "orientation": {
    "a": 0.000,                  // [°] Úhel A (rotace kolem Z). Pro G-Code vždy 0.
    "b": 0.000,                  // [°] Úhel B (rotace kolem Y). Pro G-Code vždy 0.
    "c": 0.000                   // [°] Úhel C (rotace kolem X). Pro G-Code vždy 0.
  },
  
  "kinematics": {
    "tcp_speed": 30.0            // [mm/s] Přepočtená cílová TCP rychlost (F / 60)
  },
  
  "extruder": {
    "e_per_mm": 1.250,           // [mm/mm] Extruze úměrná ujeté vzdálenosti (ΔE / sqrt(ΔX² + ΔY² + ΔZ²))
    "temperature": 240           // [°C] Aktuálně požadovaná teplota
  },
  
  "hardware": {
    "fan_speed_percent": 100     // [%] Výkon ofuku (0-100)
  },
  
  "slicer_context": {
    "layer_index": 1,            // [Int] Číslo vrstvy (inkrementováno při komentáři ;LAYER_CHANGE)
    "feature_type": 1            // [Enum/Int] Typ dráhy z komentáře ;TYPE: (0=N/A, 1=Perimeter, 2=Infill, 3=Support...)
  },
  
  "status": {
    "progress_percent": 14       // [%] Aktuální průběh generování (z M73 P)
  }
}
```

### Zásadní výpočty parseru:
1.  **Dávkování materiálu (`e_per_mm`)**: Toto eliminuje stavovost G-kodu a nutnost robotů hlídat odjetou dráhu v reálném čase. Robotický postprocesor jednoduše přikáže PLC výstupu: `Otáčky Extruderu = Rychlost_TCP_v_Dane_Milisekunde * e_per_mm`.
2.  **Identifikace `type`**:
    *   Pokud `ΔE > 0` a fyzická XYZ vzdálenost `> 0` ➔ `PRINT`.
    *   Pokud `ΔE == 0` a fyzická XYZ vzdálenost `> 0` ➔ `TRAVEL`.
    *   Pokud `ΔE < 0` a fyzická XYZ vzdálenost `== 0` ➔ `RETRACT`.
    *   Pokud `ΔE > 0` a fyzická XYZ vzdálenost `== 0` ➔ `UNRETRACT`.

---

## 3. Formát Úložiště (Kompaktní CSV Exporter)
Pro komunikaci s průmyslovými systémy (nahrávání hrubých dat do paměťových databází řídicích jednotek robota, tzv. Point listů) je struktura z bodu 2 exportována do pevného CSV formátu odděleného středníkem.

Tento přístup odstraňuje veškerou redundanci a stringové slovníky klasického JSONu – jedná se o surové sekvenční pole (pole of structures). Řídící program robota pouze rozparsuje řádek podle delimiterů na dedikovaný index paměti (`SPLIT(row, ';', &BufferArray)`).

### Schéma Hlavičky a Dat:
První řádky (začínající `#`) jsou brány jako komentář/metadata. Ostrá data začínají vždy od 1. sloupce, každá hodnota odpovídá jedné fixní pozici pole.

`TYPE ; X ; Y ; Z ; A ; B ; C ; TCP_SPEED ; E_RATIO ; TEMP ; FAN ; LAYER ; FEATURE ; PROGRESS`

**Definice sloupců pro CSV**:
1.  `TYPE` [Char]: Akce (`P`=Print, `T`=Travel, `R`=Retract, `U`=Unretract).
2.  `X` [Float]: Koncová X souřadnice [mm].
3.  `Y` [Float]: Koncová Y souřadnice [mm].
4.  `Z` [Float]: Koncová Z souřadnice [mm].
5.  `A` [Float]: Úhel A (rotace kolem Z) [°]. Z G-Code vždy 0.
6.  `B` [Float]: Úhel B (rotace kolem Y) [°]. Z G-Code vždy 0.
7.  `C` [Float]: Úhel C (rotace kolem X) [°]. Z G-Code vždy 0.
8.  `TCP_SPEED` [Float]: Rychlost ramene [mm/s].
9.  `E_RATIO` [Float]: Konstantní tok extruderu [mm extruderu na 1 mm ujeté prostorové dráhy hlavy].
10. `TEMP` [Int]: Teplota trysky [°C].
11. `FAN` [Int]: Výkon ofuku [%].
12. `LAYER` [Int]: Index aktuální tiskové vrstvy.
13. `FEATURE` [Int]: Zakódovaný typ tisku z G-Code komentáře ;TYPE (např. 1 pro perimetr, 2 pro výplň). Umožňuje postprocesoru měnit dynamiku robota v zatáčkách.
14. `PROGRESS` [Int]: Fyzický průběh tisku [%].

### Příklad Výstupního Souboru:
```csv
# Universal 3D Print Vector File v1.0
# Generator: PostSlicer
# TYPE;X;Y;Z;A;B;C;TCP_SPEED;E_RATIO;TEMP;FAN;LAYER;FEATURE;PROGRESS
P;327.041;369.423;1.000;0.000;0.000;0.000;30.0;1.250;240;100;1;1;14
P;326.640;375.953;1.000;0.000;0.000;0.000;30.0;1.250;240;100;1;1;14
P;325.423;382.381;1.000;0.000;0.000;0.000;30.0;1.250;240;100;1;1;14
T;320.000;382.381;1.000;0.000;0.000;0.000;130.0;0.000;240;100;1;0;14
R;320.000;382.381;1.000;0.000;0.000;0.000;0.0;-4.000;240;100;1;0;14
T;150.000;150.000;5.000;0.000;0.000;0.000;130.0;0.000;240;100;1;0;15
```
