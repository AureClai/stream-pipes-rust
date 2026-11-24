# A47 Givors (OSM) — rapport de construction

Généré par `build/` à partir de `raw/`. Jour simulé : 2026-06-11, 05:00–22:00.

## Réseau

Chaussées OSM : vers Lyon 10175 m retenus sur 11769 m, vers Saint-Étienne 10101 m sur 11426 m.

### Bretelles vers Lyon

| Lien | Type | Libellé OSM | Longueur | Voies |
|---|---|---|---|---|
| 12 | sortie | sortie Givors-Ouest, Saint-Romain en Gier | 222 m | 1 |
| 13 | entrée | entrée | 200 m | 1 |
| 14 | entrée | entrée | 229 m | 1 |
| 15 | sortie | sortie Givors - Centre | 284 m | 1 |
| 16 | sortie | sortie Givors - Les Plaines, Grigny | 173 m | 1 |
| 17 | entrée | entrée | 215 m | 1 |
| 18 | sortie | sortie Chasse-sur-Rhône | 464 m | 1 |
| 19 | entrée | entrée | 364 m | 1 |
| 20 | sortie | sortie Vienne, Valence, Marseille | 461 m | 1 |
| 21 | sortie | sortie A46, A7 | 118 m | 2 |

### Bretelles vers Saint-Étienne

| Lien | Type | Libellé OSM | Longueur | Voies |
|---|---|---|---|---|
| 45 | sortie | sortie Chasse-sur-Rhône | 332 m | 1 |
| 46 | entrée | entrée | 235 m | 1 |
| 47 | sortie | sortie Givors-Centre, Grigny | 125 m | 1 |
| 48 | entrée | entrée | 107 m | 1 |
| 49 | entrée | entrée | 251 m | 1 |
| 50 | sortie | sortie Givors-Montrond,  Mornant | 273 m | 1 |
| 51 | sortie | sortie Givors Ouest | 277 m | 1 |
| 52 | entrée | entrée | 226 m | 1 |

Station MYR69.73 (3262, 4.72533, 45.57855) écartée : à 43 m du réseau (lien le plus proche : 52).

Station MYR69.74 (3264, 4.73473, 45.57927) écartée : à 437 m du réseau (lien le plus proche : 51).

### Dépistage des stations

- **MYR69.31** (3390 vers Saint-Étienne) écartée de la construction de la demande : +300 véh/h depuis MYR69.51, puis -413 véh/h jusqu'à MYR69.71, sans bretelle entre elles. Elle est aussi écartée de `observed.json` : une donnée qui échoue au dépistage ne juge pas le modèle.

### Dépistage des fenêtres

- MYR69.A2 vers Lyon 15:30 : -1142 véh/h depuis MYR69.62 (habituel -252), en fluide — fenêtre écartée de la validation et de la demande.
- MYR69.A2 vers Lyon 16:00 : -1124 véh/h depuis MYR69.62 (habituel -270), en fluide — fenêtre écartée de la validation et de la demande.
- MYR69.A2 vers Lyon 16:30 : -1024 véh/h depuis MYR69.62 (habituel -262), en fluide — fenêtre écartée de la validation et de la demande.
- MYR69.A2 vers Lyon 17:00 : -1014 véh/h depuis MYR69.62 (habituel -281), en fluide — fenêtre écartée de la validation et de la demande.

## Stations

| Station | AVATAR | Sens | Lien | Écart au tracé | Fenêtres mesurées |
|---|---|---|---|---|---|
| 2224 | 2973 | bretelle | 47 | 0 m | 32/34 |
| MYR69.62 | 3068 | vers Lyon | 6 | 0 m | 33/34 |
| MYR69.63 | 3069 | bretelle | 49 | 0 m | 0/17 |
| MYR69.64 | 3070 | bretelle | 15 | 0 m | 17/17 |
| MYR69.61 | 3071 | vers Saint-Étienne | 30 | 0 m | 33/34 |
| MYR69.51 | 3109 | vers Saint-Étienne | 34 | 0 m | 33/34 |
| MYR69.52 | 3110 | vers Lyon | 3 | 1 m | 33/34 |
| MYR69.41 | 3119 | vers Saint-Étienne | 33 | 1 m | 33/34 |
| MYR69.42 | 3120 | vers Lyon | 4 | 0 m | 33/34 |
| MYR69.91 | 3229 | vers Saint-Étienne | 44 | 1 m | 33/34 |
| MYR69.71 | 3261 | vers Saint-Étienne | 36 | 0 m | 33/34 |
| MYR69.72 | 3263 | vers Lyon | 1 | 0 m | 33/34 |
| MYR69.75 | 3265 | bretelle | 51 | 0 m | 17/17 |
| MYR69.76 | 3266 | bretelle | 12 | 0 m | 17/17 |
| MYR69.77 | 3267 | bretelle | 52 | 1 m | 0/34 |
| MYR69.78 | 3268 | bretelle | 13 | 1 m | 0/34 |
| MYR69.32 | 3389 | vers Lyon | 2 | 1 m | 33/34 |
| MYR69.31 | 3390 | vers Saint-Étienne | 35 | 0 m | 0/34 |
| MY469.A2 | 3428 | vers Lyon | 9 | 1 m | 0/34 |
| MY469.A1 | 3429 | vers Saint-Étienne | 24 | 0 m | 0/34 |
| MYR69.A4 | 3446 | bretelle | 16 | 1 m | 17/17 |
| MYR69.A2 | 3447 | vers Lyon | 7 | 2 m | 29/34 |
| MYR69.A5 | 3449 | bretelle | 48 | 1 m | 0/17 |
| MYR69.A3 | 3450 | bretelle (contrôle croisé) | 47 | 1 m | écartée : 395 véh/h contre 494 pour 2224 sur la même bretelle |
| MYR69.A1 | 3451 | vers Saint-Étienne | 28 | 0 m | 33/34 |

Fenêtres d'observation retenues (entièrement mesurées) : 492/714. Les valeurs imputées par AVATAR sont exclues.

## Demande

Fenêtres de 30 min. Débits de section courante : stations ; bretelles mesurées : stations (pas horaire) ; bretelles non mesurées : conservation entre deux stations quand une seule est inconnue, sinon 0 (signalé).

| Bretelle | Libellé | Débit moyen | Source |
|---|---|---|---|
| 12 | sortie Givors-Ouest, Saint-Romain en Gier | 76 véh/h | mesurée |
| 13 | entrée | 53 véh/h | conservation |
| 14 | entrée | 542 véh/h | conservation |
| 15 | sortie Givors - Centre | 48 véh/h | mesurée |
| 16 | sortie Givors - Les Plaines, Grigny | 118 véh/h | mesurée |
| 17 | entrée | 0 véh/h | non observée : 0 |
| 18 | sortie Chasse-sur-Rhône | 0 véh/h | non observée : 0 |
| 19 | entrée | 0 véh/h | non observée : 0 |
| 20 | sortie Vienne, Valence, Marseille | 0 véh/h | répartition par voies |
| 21 | sortie A46, A7 | 0 véh/h | répartition par voies |
| 45 | sortie Chasse-sur-Rhône | 0 véh/h | non observée : 0 |
| 46 | entrée | 0 véh/h | non observée : 0 |
| 47 | sortie Givors-Centre, Grigny | 628 véh/h | mesurée |
| 48 | entrée | 130 véh/h | conservation |
| 49 | entrée | 0 véh/h | conservation (partagée) |
| 50 | sortie Givors-Montrond,  Mornant | 338 véh/h | conservation (partagée) |
| 51 | sortie Givors Ouest | 177 véh/h | mesurée |
| 52 | entrée | 362 véh/h | conservation |

Contrôle de conservation entre stations sans bretelle inconnue : écart moyen absolu 74 véh/h sur 166 fenêtres.

### Contrôle station par station

Variation de débit entre deux stations consécutives (aval − amont), moyenne sur les fenêtres où les deux sont mesurées.

| Sens | Amont | Aval | Bretelles entre | Δ moyen | |Δ| moyen |
|---|---|---|---|---|---|
| vers Lyon | 72 | 32 | 13 | +35 | 71 |
| vers Lyon | 32 | 52 | — | -11 | 40 |
| vers Lyon | 52 | 42 | — | -25 | 55 |
| vers Lyon | 42 | 62 | 14, 15 | +495 | 495 |
| vers Lyon | 62 | A2 | 16 | -215 | 215 |
| vers Saint-Étienne | A1 | 61 | 48 | +130 | 130 |
| vers Saint-Étienne | 61 | 41 | 49, 50 | -338 | 338 |
| vers Saint-Étienne | 41 | 51 | — | -5 | 48 |
| vers Saint-Étienne | 51 | 71 | — | -113 | 115 |
| vers Saint-Étienne | 71 | 91 | 51, 52 | +185 | 187 |

Demande : 669 lignes, 88611 véhicules.

## Condition aval (patch `gate`)

La station MYR69.A2 est congestionnée (< 50 km/h) sur 24 pas de 6 min : pendant ces pas, le lien 11 (fin de l'A47 vers Lyon) prend pour capacité le débit qu'elle mesure. Sans ce patch, le goulot de la jonction A7, hors du réseau, n'existe pas.

## Liens

| Lien | Sens | Type | Longueur | Voies | Limite | u (km/h) |
|---|---|---|---|---|---|---|
| 0 | vers Lyon | Mainline | 4234 m | 2 | 90 | 103 |
| 1 | vers Lyon | Mainline | 455 m | 2 | 90 | 103 |
| 2 | vers Lyon | Mainline | 1606 m | 2 | 90 | 97 |
| 3 | vers Lyon | Mainline | 552 m | 2 | 90 | 91 |
| 4 | vers Lyon | Mainline | 256 m | 2 | 90 | 91 |
| 5 | vers Lyon | Mainline | 896 m | 2 | 70 | 77 |
| 6 | vers Lyon | Mainline | 232 m | 2 | 70 | 77 |
| 7 | vers Lyon | Mainline | 280 m | 2 | 70 | 82 |
| 8 | vers Lyon | Mainline | 539 m | 2 | 70 | 82 |
| 9 | vers Lyon | Mainline | 354 m | 2 | 70 | 82 |
| 10 | vers Lyon | Mainline | 703 m | 2 | 70 | 82 |
| 11 | vers Lyon | Mainline | 70 m | 3 | 70 | 82 |
| 12 | vers Lyon | OffRamp | 222 m | 1 | 70 | 70 |
| 13 | vers Lyon | OnRamp | 200 m | 1 | 70 | 70 |
| 14 | vers Lyon | OnRamp | 229 m | 1 | 90 | 90 |
| 15 | vers Lyon | OffRamp | 284 m | 1 | 70 | 70 |
| 16 | vers Lyon | OffRamp | 173 m | 1 | 70 | 70 |
| 17 | vers Lyon | OnRamp | 215 m | 1 | 50 | 50 |
| 18 | vers Lyon | OffRamp | 464 m | 1 | 50 | 50 |
| 19 | vers Lyon | OnRamp | 364 m | 1 | 50 | 50 |
| 20 | vers Lyon | OffRamp | 461 m | 1 | 70 | 70 |
| 21 | vers Lyon | OffRamp | 118 m | 2 | 70 | 70 |
| 22 | vers Saint-Étienne | Mainline | 435 m | 2 | 70 | 78 |
| 23 | vers Saint-Étienne | Mainline | 124 m | 3 | 70 | 78 |
| 24 | vers Saint-Étienne | Mainline | 253 m | 2 | 70 | 78 |
| 25 | vers Saint-Étienne | Mainline | 192 m | 3 | 70 | 78 |
| 26 | vers Saint-Étienne | Mainline | 462 m | 2 | 70 | 78 |
| 27 | vers Saint-Étienne | Mainline | 96 m | 3 | 70 | 78 |
| 28 | vers Saint-Étienne | Mainline | 128 m | 2 | 70 | 78 |
| 29 | vers Saint-Étienne | Mainline | 123 m | 3 | 70 | 78 |
| 30 | vers Saint-Étienne | Mainline | 251 m | 2 | 70 | 78 |
| 31 | vers Saint-Étienne | Mainline | 808 m | 2 | 70 | 83 |
| 32 | vers Saint-Étienne | Mainline | 88 m | 3 | 70 | 83 |
| 33 | vers Saint-Étienne | Mainline | 291 m | 2 | 70 | 83 |
| 34 | vers Saint-Étienne | Mainline | 552 m | 2 | 90 | 85 |
| 35 | vers Saint-Étienne | Mainline | 733 m | 2 | 90 | 94 |
| 36 | vers Saint-Étienne | Mainline | 652 m | 2 | 90 | 96 |
| 37 | vers Saint-Étienne | Mainline | 164 m | 3 | 90 | 96 |
| 38 | vers Saint-Étienne | Mainline | 498 m | 2 | 90 | 96 |
| 39 | vers Saint-Étienne | Mainline | 163 m | 3 | 90 | 96 |
| 40 | vers Saint-Étienne | Mainline | 620 m | 2 | 90 | 96 |
| 41 | vers Saint-Étienne | Mainline | 142 m | 3 | 90 | 96 |
| 42 | vers Saint-Étienne | Mainline | 483 m | 2 | 90 | 96 |
| 43 | vers Saint-Étienne | Mainline | 68 m | 3 | 90 | 96 |
| 44 | vers Saint-Étienne | Mainline | 2775 m | 2 | 90 | 97 |
| 45 | vers Saint-Étienne | OffRamp | 332 m | 1 | 70 | 70 |
| 46 | vers Saint-Étienne | OnRamp | 235 m | 1 | 50 | 50 |
| 47 | vers Saint-Étienne | OffRamp | 125 m | 1 | 50 | 50 |
| 48 | vers Saint-Étienne | OnRamp | 107 m | 1 | 70 | 70 |
| 49 | vers Saint-Étienne | OnRamp | 251 m | 1 | 70 | 70 |
| 50 | vers Saint-Étienne | OffRamp | 273 m | 1 | 70 | 70 |
| 51 | vers Saint-Étienne | OffRamp | 277 m | 1 | 50 | 50 |
| 52 | vers Saint-Étienne | OnRamp | 226 m | 1 | 70 | 70 |
