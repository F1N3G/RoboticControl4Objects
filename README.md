# README.md – Optimizarea Traiectoriei unui Braț Robotic

## 📊 Obiectivul proiectului
Scopul acestui proiect este simularea și optimizarea traiectoriei unui braț robotic plan (2D) cu două segmente de lungimi L1 și L2. Obiectivul principal este de a compara trei metode diferite de generare a traiectoriei capătului liber (end-effector):

1. **Cinematica inversă directă** – folosind o traiectorie lină (linie dreaptă);
2. **Optimizare prin metoda Least Squares** – ajustare matematică pe baza datelor experimentale;
3. **Optimizare prin metoda DFP (Davidon-Fletcher-Powell)** – cu scopul de a minimiza variația unghiurilor brațului robotic pentru o mișcare fluidă.

Toate traiectoriile sunt animate și comparate vizual folosind grafică MATLAB.

Acest proiect poate fi folosit ca studiu de caz pentru învățarea cinematicei inverse, a regresiei liniare și a metodelor de optimizare neliniară în robotică. Este util atât pentru cursurile de „Tehnici de Optimizare”, cât și pentru dezvoltarea sistemelor inteligente de control robotic.

---

## ⚙️ Structura codului principal (`Test3.m`)

### 1. Definirea brațului robotic
```matlab
L1 = 5; % Lungimea primului segment
L2 = 3; % Lungimea celui de-al doilea segment
```
Acestea sunt constantele sistemului robotic 2D cu două articulații (2 DOF).

---

### 2. Traiectoria normală – Cinematica inversă simplă
Se generează 100 de puncte între două coordonate folosind `linspace`. Pentru fiecare punct (x, y), se calculează unghiurile articulațiilor astfel încât capătul brațului să ajungă în acel punct:

#### Teorie: Cinematica inversă
Aceasta presupune calcularea unghiurilor articulațiilor unui braț robotic pentru a atinge o poziție dată a capătului liber.

Se folosește legea cosinusului pentru a determina unghiurile articulațiilor:
```matlab
c2 = (x^2 + y^2 - L1^2 - L2^2) / (2 * L1 * L2);
s2 = sqrt(1 - c2^2);
theta2 = atan2(s2, c2);
```
Apoi se determină θ1:
```matlab
k1 = L1 + L2 * cos(theta2);
k2 = L2 * sin(theta2);
theta1 = atan2(y, x) - atan2(k2, k1);
```
Astfel, obținem un set complet de unghiuri pentru fiecare punct.

> Aceste relații permit calculul invers al configurației robotului pornind de la poziția dorită a efectorului.

---

### 3. Traiectoria optimizată prin metoda Least Squares
Se lucrează cu un set de puncte `(x_data, y_data)` pentru care se dorește o traiectorie apropiată:
```matlab
A = [x_data, ones(size(x_data))];
coef = (A' * A) \ (A' * y_data);
a = coef(1); b = coef(2);
y = a * x + b;
```

#### Teorie: Metoda celor mai mici pătrate (Least Squares)
- Este folosită pentru a aproxima soluția unui sistem `Ax = b` când sistemul este **supra-determinat** (mai multe ecuații decât necunoscute);
- Se minimizează eroarea pătratică `‖Ax - b‖²` între datele observate și modelul matematic presupus;
- Soluția exactă este dată de:
```matlab
x* = (AᵗA)⁻¹Aᵗb
```
Aceasta oferă o ecuație de regresie `y = ax + b` care se potrivește cel mai bine punctelor oferite.

📚 Vezi Laborator 6+7 pentru demonstrații și exemple complete.

> Această metodă este esențială în modelare și estimare în toate domeniile inginerești.

---

### 4. Traiectorie optimizată prin DFP – Optimizare neliniară

Se folosește o funcție cost care urmărește:
- Să reducă variația unghiurilor θ1 între pozițiile succesive ale brațului;
- Să mențină punctele cât mai apropiate de traiectoria originală definită de `(x_data, y_data)`.

```matlab
fun = @(v) cost_variatie_unghiuri(v, L1, L2, x_data, y_data);
sol = fminunc(fun, x0, options);
```

#### Teorie: Metoda DFP (Davidon-Fletcher-Powell)
- Este o **metodă Quasi-Newton** care evită calculul Hessienei (matricea derivatelor de ordin 2);
- În schimb, folosește variațiile gradientului pentru a construi o aproximare a Hessienei inverse:
```matlab
Hk+1 = Hk + (Δx * Δxᵗ) / (Δxᵗ * Δg) - (Hk * Δg * Δgᵗ * Hk) / (Δgᵗ * Hk * Δg)
```
- Asigură direcții de descendență și stabilitate numerică;
- Utilizată pentru probleme neliniare, fără constrângeri;
- În MATLAB se folosește `fminunc` cu opțiunea `quasi-newton`.

🔎 Detalii în Laborator 6+7, paginile 14–18.

> DFP face parte dintr-o clasă de algoritmi extrem de eficienți pentru probleme de optimizare fără constrângeri, fiind o alternativă performantă la metodele clasice Newton.

---

### 5. Funcția `cost_variatie_unghiuri(...)`
Această funcție calculează valoarea costului pentru optimizare:
```matlab
J = sum(diff(theta).^2) + 10 * sum((x - x_ref_interp).^2 + (y - y_ref_interp).^2);
```
- `sum(diff(theta).^2)` – penalizează schimbările bruște ale unghiurilor θ1;
- `sum((x - x_ref_interp).^2 + (y - y_ref_interp).^2)` – penalizează devierea de la traiectoria de referință.

Este o combinație între optimizare a traiectoriei geometrice și a mișcării articulațiilor.

> Acest cost este critic pentru obținerea unei mișcări naturale și eficiente energetic a brațului robotic.

---

## 🎬 Animația brațului robotic
Funcția `animeaza_brat(...)` este folosită pentru a vizualiza fiecare traiectorie:
- Desenează segmentele brațului cu culori diferite;
- Actualizează pozițiile la fiecare pas;
- Include punctele de articulare (bază, cot, capăt).

```matlab
animeaza_brat(x_traj, y_traj, L1, L2, 'Titlu');
```
Este apelată de 3 ori pentru fiecare metodă:
1. Cinematică inversă;
2. Least Squares;
3. DFP.

> Vizualizarea este esențială pentru înțelegerea practică a comportamentului optimizat al brațului.

---

## 📚 Teorie necesară pentru înțelegerea completă

| Concept | Descriere |
|--------|-----------|
| Cinematică inversă | Calculul unghiurilor pentru a atinge o poziție (x, y) dată |
| Least Squares | Ajustare matematică ce minimizează eroarea între date și model |
| Optimizare Quasi-Newton | Algoritm iterativ eficient pentru minimizarea funcțiilor neliniare |
| DFP | O metodă Quasi-Newton ce aproximează inversa Hessienei |

---

## 🧠 Interpretare practică
- **Cinematica inversă** → simplă, dar poate cauza mișcări sacadate;
- **Least Squares** → traiectorie netedă, dar nu ține cont de unghiuri;
- **DFP** → mișcare realistă, optimizată pentru sistem robotic fizic.

Aplicațiile reale includ:
- Brațe robotice colaborative în industrie;
- Sisteme de chirurgie robotică;
- Manipulare de obiecte fragile în medii dinamice.

---

## 📈 Recomandări pentru prezentare
- Explică clar diferențele între metode (în special DFP vs. Least Squares);
- Demonstrează animațiile în paralel pentru efect vizual;
- Menționează aplicații practice: chirurgie robotică, manipulare precisă etc.;
- Adaugă un slide despre teoria DFP cu ecuația completă a actualizării Hessienei inverse;
- Folosește grafice comparative pentru traiectorii și unghiuri.

---

## 👥 Membrii echipei
- Florescu Neagoe Gabriela Alexandra – dezvoltare metodă DFP, animații, documentație
- Pogor Mihai – regresie Least Squares, partea teoretică
- Vilceanu Lucia Alexandra – cinematică inversă, grafică
- Braicu Raluca - testare și implementare

---

## 📦 Structura pentru GitHub
```
/
├── README.md
├── Test3.m
└── /doc
    ├── Laborator 6+7.pdf
    └── capturi_grafic.png
```

---

## 🌟 Posibile extinderi
- Compararea cu metode BFGS sau Nelder-Mead
- Introducerea de obstacole în spațiul de lucru
- Control în timp real cu joystick sau input live
- Integrare cu senzori pentru feedback vizual sau forță

---

## 🔚 Concluzie
Proiectul arată cum se pot integra metode matematice și algoritmi de optimizare pentru a obține mișcări mai inteligente și naturale ale brațelor robotice. Prin implementarea corectă a cinematicei și optimizării, se pot reduce erorile, costurile energetice și se poate crește precizia manipulării în aplicații reale. Totodată, se formează o bază solidă pentru înțelegerea aprofundată a tehnicilor moderne de optimizare aplicate în robotică, inteligentă artificială și mecatronică.

clc; clear; close all;

% === Parametri braț robotic ===
L1 = 5;
L2 = 3;

% === Traiectorie normală ===
x_norm = linspace(2, 6, 100);
y_norm = linspace(3, 5, 100);

% === Traiectorie optimizată prin Least Squares ===
x_data = [2, 3, 4, 5, 6]';
y_data = [3, 3.5, 4.1, 4.6, 5]';
A = [x_data, ones(size(x_data))];
coef = (A' * A) \ (A' * y_data);
a = coef(1); b = coef(2);
x_opt1 = linspace(min(x_data), max(x_data), 100);
y_opt1 = a * x_opt1 + b;

% === Traiectorie optimizată prin DFP (minimizăm variația unghiurilor) ===
x_init = linspace(2, 6, 20);
y_init = linspace(3, 5, 20);
x0 = [x_init; y_init];
x0 = x0(:);

fun = @(v) cost_variatie_unghiuri(v, L1, L2, x_data, y_data);
options = optimoptions('fminunc', 'Algorithm', 'quasi-newton', 'Display', 'none', 'MaxIterations', 500);
sol = fminunc(fun, x0, options);

sol = reshape(sol, 2, []);
x_opt2 = sol(1, :);
y_opt2 = sol(2, :);

% === Funcție generală pentru animație ===
function animeaza_brat(x_traj, y_traj, L1, L2, titlu)
    figure; axis([-10 10 -10 10]);
    hold on; grid on;
    xlabel('X'); ylabel('Y'); title(titlu);

    h1 = plot([0, 0], [0, 0], 'ro-', 'LineWidth', 3);
    h2 = plot([0, 0], [0, 0], 'bo-', 'LineWidth', 3);
    base = plot(0, 0, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
    joint1 = plot(0, 0, 'ko', 'MarkerFaceColor', 'g');
    joint2 = plot(0, 0, 'ko', 'MarkerFaceColor', 'b');

    for i = 1:length(x_traj)
        xe = x_traj(i); ye = y_traj(i);
        c2 = (xe^2 + ye^2 - L1^2 - L2^2) / (2 * L1 * L2);
        if abs(c2) > 1
            continue;
        end
        s2 = sqrt(1 - c2^2);
        theta2 = atan2(s2, c2);
        k1 = L1 + L2 * cos(theta2);
        k2 = L2 * sin(theta2);
        theta1 = atan2(ye, xe) - atan2(k2, k1);

        x1 = L1 * cos(theta1); y1 = L1 * sin(theta1);
        x2 = x1 + L2 * cos(theta1 + theta2);
        y2 = y1 + L2 * sin(theta1 + theta2);

        set(h1, 'XData', [0 x1], 'YData', [0 y1]);
        set(h2, 'XData', [x1 x2], 'YData', [y1 y2]);
        set(joint1, 'XData', x1, 'YData', y1);
        set(joint2, 'XData', x2, 'YData', y2);

        pause(0.05);
    end
    hold off;
end

% === Funcție cost pentru DFP ===
function J = cost_variatie_unghiuri(v, L1, L2, x_ref, y_ref)
    p = reshape(v, 2, []);
    x = p(1, :);
    y = p(2, :);
    theta = zeros(1, length(x));
    J = 0;
    for i = 1:length(x)
        xe = x(i); ye = y(i);
        c2 = (xe^2 + ye^2 - L1^2 - L2^2) / (2 * L1 * L2);
        if abs(c2) > 1
            J = inf; return;
        end
        s2 = sqrt(1 - c2^2);
        theta2 = atan2(s2, c2);
        k1 = L1 + L2 * cos(theta2);
        k2 = L2 * sin(theta2);
        theta1 = atan2(ye, xe) - atan2(k2, k1);
        theta(i) = theta1;
    end
    % Cost total: variație unghiuri + abatere față de traiectoria inițială
    J = sum(diff(theta).^2) + 10 * sum((x - interp1(1:length(x_ref), x_ref, linspace(1, length(x_ref), length(x)))).^2 + ...
        (y - interp1(1:length(y_ref), y_ref, linspace(1, length(y_ref), length(y)))).^2);
end

% === Rularea animațiilor ===
animeaza_brat(x_norm, y_norm, L1, L2, 'Traiectorie Cinematica Inversa (normală)');
pause(2);
animeaza_brat(x_opt1, y_opt1, L1, L2, 'Traiectorie Optimizată (Least Squares)');
pause(2);
animeaza_brat(x_opt2, y_opt2, L1, L2, 'Traiectorie Optimizată (DFP - cu constrângere de proximitate)');
