# KUKA 3D Print Toolkit

Stav dokumentace: 4. 3. 2026

Tento repozitar je zaklad pro 3D tisk s robotem KUKA. Projekt je rozdeleny na dve hlavni casti:

- `Python` cast (univerzalni): import trajektorii z vice formatu, prevod do jednotneho CSV, 3D viewer, simulace dosazitelnosti a export pres postprocesory.
- `KRL` cast v `Interpret` (KUKA specificka): streamovani bodu ze souboru pres SPS na pozadi a jejich postupne provadeni robotem bez pevneho limitu poctu bodu.

## Co projekt umi

### 1) Univerzalni pipeline v Pythonu

- Konverze z `G-code` (typicky PrusaSlicer) do univerzalniho CSV: `gcode_to_csv.py`
- Konverze z `NC` G-code (typicky M3/M5 + S) do stejneho CSV: `nc_to_csv.py`
- Automaticka detekce vstupu (`.gcode/.gco/.nc`) ve vieweru a vyber spravneho parseru
- Vizualizace trajektorie v 3D (`csv_viewer_pyvista.py`) s rychlym vykreslovanim:
- tiskove useky jako "tube" geometrie
- travel/retract useky jako tenke cary
- Posuvnik po bodech a po vrstvach
- Odhad casu tisku z delky segmentu a `TCP_SPEED`
- Nacteni URDF robota a IK sledovani aktualniho bodu trajektorie
- Nastaveni pracoviste:
- stul (rozmery/pozice)
- BASE frame robota (X,Y,Z,A,B,C)
- TOOL frame (X,Y,Z,A,B,C)
- Test trajektorie pres vsechny body v paralelnim rezimu:
- status `OK`
- prekroceni limitu os
- singularita zapesti (A5 blizko 0 deg)
- nedosazitelny bod
- Editace trajektorie ve vieweru:
- posun X/Y
- rotace kolem Z kolem teziste modelu
- ulozeni zpet do CSV
- Export pres postprocesory ze slozky `Postprocesor`

### 2) Postprocesory

Aktualne jsou v repozitari:

- `Postprocesor/1_to_1_export.py`
- Testovaci, prakticky jen kopie CSV do textu (CRLF).
- `Postprocesor/null.py`
- Prazdny placeholder.
- `Postprocesor/JedenRadekDvanactSloupcu.py`
- Produkcni format: 1 radek = 1 bod, vhodny pro `Interpret/cnc.src + sps.sub + coin2.src`.
- `Postprocesor/DvaRadkyOsmSloupcu.py`
- Produkcni/alternativni format: 2 radky na bod (hlavicka + data), odpovida starsi vetvi `cnc2/coin4`.

### 3) KUKA cast v `Interpret`

- `sps.sub`:
- bezici background task
- obsluha OPEN/CLOSE/RESET prikazu
- cteni souboru po radcich (`krl_fgets`) a parsovani do ring-bufferu
- handshake pres `REQ_ID/ACK_ID`
- vystup rychlosti na PLC (`s_RP_MV_Spd`) po prepoctu aktualni rychlosti robota
- `cnc.dat`:
- globalni struktury a sdilena data
- ring-buffer bodu (`CNC_BUF_POINTS`)
- trigger buffer payloadu (`CNC_TRG_BUF`)
- `cnc.src`:
- knihovna funkci `Init`, `CncOpen`, `CncClose`, `CncGetNextPoint`
- pohybove funkce `MoveLin` a `MoveLinExact`
- trigger callback `CncOnPointStart`
- `coin2.src`:
- ukazkovy hlavni program tisku
- otevre stream (`coin.txt`)
- nacita body prubezne
- provadi plynule `LIN ... C_DIS` a posledni bod `LIN` presne
- umi projet velmi dlouhe trajektorie bez nutnosti drzet vse v jednom SRC programu

## Co projekt zatim neumi (realne limity)

- Ve vieweru neni implementovana skutecna detekce kolizi robota:
- neni self-collision check
- neni kolize s okolim/stolem/nastrojem
- test je dnes IK + limity + singularita, nikoli plna kolizni analyza.
- Sloupce orientace `A/B/C` z CSV se nacitaji, ale viewer je zatim nepouziva jako per-bod orientaci TCP.
- `gcode_to_csv.py` je parser pro bezne slicer G-code, ale ne plna implementace vsech dialektu:
- neresi oblouky `G2/G3`
- nepodporuje relativni extrusion workflow jako plnohodnotny stavovy automat
- je zameren hlavne na `G1`, `G92 E0`, `M104/M109`, `M106/M107`, `M73`, komentare `;TYPE` a `;LAYER_CHANGE`
- `nc_to_csv.py` predpoklada styl s `M3/M5` a standalone `S...`; jine konvence mohou vyzadovat upravu parseru.
- Neni zde sada automatickych testu ani CI pipeline.
- Chybi centralni instalacni skript zavislosti (`requirements.txt`/`pyproject.toml`).

## Co od projektu cekat

- Je to funkcni zaklad pro prakticky workflow:
- import -> univerzalni CSV -> vizualni kontrola -> postprocesor -> stream na KUKA.
- Je vhodny jako rozsiritelna platforma:
- pridani dalsich parseru vstupu
- pridani dalsich postprocesoru (jine roboticke jazyky)
- pridani dalsich URDF robotu
- KUKA cast je navrzena tak, aby resila hlavni bolest KRL:
- neomezovat se velikosti jedne SRC trajektorie
- cist body postupne z textoveho souboru na pozadi
- Pro produkcni nasazeni je potreba doplnit:
- robustni validace vstupu
- kolizni analyzu
- testy
- standardizaci deploymentu na KRC

## Univerzalni CSV format

Hlavni schema (oddeleni `;`):

`TYPE;X;Y;Z;A;B;C;TCP_SPEED;E_RATIO;TEMP;FAN_PCT;LAYER;FEATURE;PROGRESS`

- `TYPE`: `P` print, `T` travel, `R` retract, `U` unretract
- `X/Y/Z`: pozice [mm]
- `A/B/C`: orientace [deg]
- `TCP_SPEED`: rychlost TCP [mm/s]
- `E_RATIO`: tok materialu (parser-specific; v NC vetvi se pouziva jako extruder hodnota ze `S`)
- `TEMP`, `FAN_PCT`, `LAYER`, `FEATURE`, `PROGRESS`: doplnkove technologicke informace

## Vazba postprocesor -> KUKA

### Format pro `coin2.src` (`cnc.src` + `sps.sub`)

Pouzij `JedenRadekDvanactSloupcu.py`, vystupni radek:

`X Y Z A B C V_MOVE V_EXT POINT_NO LAYER_NO PROGRESS NOTE`

To odpovida parsovani v `sps.sub` (`SREAD` ve 3 krocich).

### Format pro starsi vetev `cnc2/coin4`

Pouzij `DvaRadkyOsmSloupcu.py`:

- radek 1: textova hlavicka
- radek 2: `X Y Z A B C V_MOVE V_EXT`

## Struktura repozitare

- `csv_viewer_pyvista.py` hlavni GUI viewer
- `robot_sim.py` URDF + IK/FK vrstva
- `gcode_to_csv.py` konverze slicer G-code -> CSV
- `nc_to_csv.py` konverze NC (M3/M5/S) -> CSV
- `Postprocesor/` vystupni formaty
- `CSV/`, `GCODE/`, `Output/` ukazkova data
- `kuka_kr16_support/` URDF + meshe KR16
- `Interpret/` KRL cast (SPS, knihovna, ukazkove programy)
- `Interpret/R1/` puvodni struktura ze zalohy kontroleru (orientacne)

## Rychly start (Python cast)

1. Instaluj zavislosti (minimalne):

`PyQt6`, `numpy`, `pyvista`, `pyvistaqt`, `ikpy`, `yourdfpy`, `trimesh` (+ jejich zavislosti).

2. Prevod vstupu do CSV:

```bash
python gcode_to_csv.py GCODE/model.gcode -o CSV/model.csv
python nc_to_csv.py "GCODE/model.nc" -o CSV/model.csv
```

3. Spust viewer:

```bash
python csv_viewer_pyvista.py
```

4. Ve vieweru:

- nacti CSV
- nacti URDF (`kuka_kr16_support/urdf/kr16_2.urdf`)
- nastav BASE/TOOL
- spust `Test Trajectory`
- exportuj pres postprocesor do textu pro robota

## Rychly start (KUKA cast)

1. Nahraj odpovidajici KRL soubory do kontroleru (`Interpret` cast).
2. Umisti vygenerovany textovy soubor bodu do adresare programu (v `coin2.src` je `fileName[] = "coin.txt"`).
3. Spust SPS (`sps.sub`) a hlavni program (`coin2.src`).
4. Over mapovani signalu v `$config.dat`:

- `S_Robot_Vel` na `$ANOUT[10]`
- `s_RP_MV_Spd` na `$OUT[328..359]`
- teploty a enable signaly dle lokalniho PLC zapojeni

## Dulezita poznamka

Tento projekt je silny technicky zaklad, ne "hotovy produkt na klik". Nejvetsi hodnota je v architekture:

- jednotny meziformat trajektorie
- oddeleni importu, simulace a exportu
- streamovani bodu na KUKA bez pevneho limitu trajektorie

Prave to je pripraveno na dalsi iterace podle konkretniho robota, extruderu a PLC logiky.

