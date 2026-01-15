# Physics-Aware A* Algorithm

**English**
This project modifies the standard A* (A-Star) algorithm to optimize for **Energy Consumption** rather than distance. It includes a physics engine that calculates Gravitational Force ($F = mg \sin \theta$) and Friction ($F = \mu N$) to determine the cost of movement on 3D terrain.

**Türkçe**
Bu proje, standart A* algoritmasını mesafeyi değil, **Enerji Tüketimini** minimize edecek şekilde modifiye eder. 3D arazide hareket maliyetini belirlemek için Yerçekimi Kuvveti ve Sürtünme Kuvvetini hesaplayan bir fizik motoru içerir.

## Features / Özellikler
* Calculates energy cost in Joules. (Enerji maliyetini Joule cinsinden hesaplar.)
* Accounts for slope angles and vehicle mass. (Eğim açılarını ve araç kütlesini hesaba katar.)
* Avoids steep climbs to save battery. (Batarya tasarrufu için dik yokuşlardan kaçınır.)
