# Swerve Drive Simulator

Simulateur 2D d'un robot à deux modules swerve (centre) et quatre roues folles passives (coins), équipé d'une pince à l'avant.  
Testable directement sur mobile : **https://matthieugrr.github.io/swerve-controller/**

---

## Utilisation

| Geste | Effet |
|---|---|
| Tap sur le canvas | Définit un objectif de position |
| Appui + glisser | Définit position **et** angle d'arrivée (flèche orange) |
| 🔄 Reset | Remet le robot à l'origine |
| 🗑️ Effacer | Supprime la trajectoire |

---

## Architecture du contrôleur

La boucle tourne à **20 Hz (DT = 0.05 s)** et enchaîne trois étapes à chaque pas.

### 1 · Calcul des commandes — `computeControl`

À partir de l'erreur de position et de cap, on produit trois consignes en **repère robot** :

```
ex_r =  ex·cos θ + ey·sin θ   ← erreur projetée axe avant du robot
ey_r = -ex·sin θ + ey·cos θ   ← erreur projetée axe latéral

‖v‖ = √((Kx·ex_r)² + (Ky·ey_r)²)
vx_cmd = Kx·ex_r · min(1, MAX_VXY/‖v‖)   ← saturation vectorielle
vy_cmd = Ky·ey_r · min(1, MAX_VXY/‖v‖)      (préserve la direction)

etheta  = sawtooth(desired_heading − θ)   (normalisé dans [−π, π])
omega   = Kθ · etheta           (saturé à ±MAX_OMEGA)
```

`desired_heading` vaut :
- l'angle vers la cible (`atan2(ey, ex)`) si aucun angle d'arrivée n'est fixé
- directement `target_theta` sinon — le swerve drive ne nécessite pas de "regarder où l'on va", donc la rotation démarre dès le début du trajet, **en parallèle** de la translation

L'arrêt se produit quand `distance < 2 cm` **ET** `|θ − target_theta| < 0.1 rad` (si un cap est demandé).

---

### 2 · Cinématique inverse — `inverseKinematics`

Pour chaque module swerve à la position locale `(lx, ly)` :

```
vx_mod = vx_cmd − omega · ly
vy_mod = vy_cmd + omega · lx

speed_i = √(vx_mod² + vy_mod²)
angle_i = atan2(vy_mod, vx_mod)
```

**Optimisation d'angle** : si tourner la roue de plus de 90° est plus court en sens inverse, on inverse l'angle et le signe de la vitesse. Ça évite des rotations à 270°.

Si la vitesse max dépasse `MAX_WHEEL_SPEED`, toutes les vitesses sont normalisées proportionnellement (pas de perte de trajectoire).

**Conversion PWM** (zone morte < 1 cm/s) :
```
speed > 0 → PWM = PWM_MIN + (PWM_MAX − PWM_MIN) · |speed / MAX_WHEEL_SPEED|
speed < 0 → symétrique négatif
```
avec `PWM_MIN = 145`, `PWM_MAX = 255`.

---

### 3 · Intégration — `update`

Les commandes en repère robot sont repassées en repère monde avant intégration :

```
x += (vx·cos θ − vy·sin θ) · dt
y += (vx·sin θ + vy·cos θ) · dt
θ += omega · dt   (normalisé dans [−π, π])
```

---

## Paramètres ajustables

| Paramètre | Défaut | Rôle |
|---|---|---|
| Kx | 2.0 | Réactivité translation avant/arrière |
| Ky | 2.0 | Réactivité translation latérale |
| Kθ | 1.5 | Réactivité correction de cap |
| Vmax | 30 cm/s | Saturation des commandes de translation |

Augmenter les gains accélère la convergence mais peut provoquer des oscillations.

---

## Géométrie du robot

```
   AV-G ○────────────────────○ AV-D   ← roues folles passives (avant)
        (10,-8)          (10, 8)

   CT-G □                    □ CT-D   ← modules swerve motorisés (centre)
        (0, -8)           (0,  8)

   AR-G ○────────────────────○ AR-D   ← roues folles passives (arrière)
        (-10,-8)         (-10, 8)

   [====pince====]→   avant du robot (x+)
```

Coordonnées en cm dans le repère robot. Les modules swerve sont centrés (`lx = 0`) pour minimiser le couplage entre `vy` et `omega`. Les roues folles occupent les quatre coins.

**Visualisation** : chaque module swerve affiche une bande verte claire et un point blanc sur sa demi-roue avant, permettant de lire l'orientation en temps réel.
