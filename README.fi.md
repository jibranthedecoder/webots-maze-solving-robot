# Webots-labyrinttirobotti

Webots-robotiikan case study, jossa Thymio-tyylinen labyrinttirobotti käyttää kolmea etäisyysanturia, suuntatietoa, PID-seinäseurantaa, enkooderipohjaisia käännöksiä ja tilakonetta labyrintin läpäisyyn.

## Portfolio case study

- [Portfolio-sivu](https://www.jibranhussain.com/projects/maze-solving-robot/)
- [English README](README.md)

## Projektin yhteenveto

Tehtävänä oli kirjoittaa ohjain, jonka avulla robotti kulkee labyrintin läpi. Robotilla on etäisyysanturit vasemmalla, edessä ja oikealla sekä suuntatieto orientaation seuraamiseen.

Toteutettu ohjain yhdistää seinäseurannan ja tilakonepohjaisen navigoinnin. Robotti ajaa eteenpäin, korjaa etäisyyttään seinään PID-säädöllä, tunnistaa esteitä etäisyysantureista ja tekee kalibroituja käännöksiä pyöräenkooderien avulla.

## Järjestelmä

- Simulaatioympäristö: Webots
- Robottityyppi: Thymio-tyylinen labyrinttirobotti
- Ohjauskieli: Python
- Anturit: vasen, etu- ja oikea etäisyysanturi
- Suuntatieto: kompassi tai inertial heading -laite
- Toimilaitteet: vasen ja oikea pyörämoottori
- Ohjausmenetelmät: PID-seinäseuranta, enkooderipohjainen kääntyminen ja tilakoneohjaus

## Insinööriongelma

Labyrinttirobotin täytyy tehdä päätöksiä rajallisen anturidatan perusteella. Sen pitää pitää vakaa etäisyys seinistä, tunnistaa edessä oleva este, valita kääntymissuunta, käsitellä umpikujat ja pysähtyä, kun labyrintti on ratkaistu.

## Ohjausidea

Ohjain käyttää tilakonetta:

```text
move_fwd_max_vel -> turn_left / turn_right -> dead_end -> end
```

Eteenpäin ajaessa robotti käyttää PID-korjausta sopivan seinäetäisyyden ylläpitämiseen. Käännökset tehdään pyöräenkooderien palautteen perusteella.

## Todisteaineisto

| Todiste | Tiedosto |
|---|---|
| Python-ohjain | [`maze_solver.py`](maze_solver.py) |
| Simulaatiovideo | [`maze_simulation.mp4`](maze_simulation.mp4) |

## Mitä projekti osoittaa

- etäisyysanturipohjainen navigointi
- PID-seinäseuranta
- enkooderipohjainen käännösohjaus
- tilakonesuunnittelu
- labyrintin päätöslogiikka
- Webots-robottisimulaatio
- Python-ohjelmointi

## Toteutushuomio

Ohjain lukee suuntatietoa `getRollPitchYaw()`-metodilla laitteesta nimeltä `compass`. Tämä toimii, jos Webots-maailmassa kyseinen laite tarjoaa inertial heading -tyyppisen rajapinnan. Jos maailma käyttää tavallista Webots `Compass` -laitetta, suuntatiedon luku pitää muuttaa käyttämään `getValues()`-metodia.

## Lisenssihuomio

Lähdekoodi on julkaistu MIT-lisenssillä. Raportti ja mediatiedostot ovat mukana portfolio-todisteina ja opetuksellisena demonstraationa.
