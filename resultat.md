# Coding-challenge-2023-Path-planning

## Tanke bakom koden

Metoden bygger på att ett visst antal punkter (inklusive start- och målpunkt) tas som stickprov i det område som begränsar problemet. Punkterna sammanlänkas sen med villkoret att en sammanlänkning inte får skära genom en hinderkub. Alla dessa sammanlänkningar bildar ett nätverk, en graf. Denna graf analyseras med Dijkstra's algoritm för att hitta den korste vägen från start till mål. Svårigheten med metoden är att hitta ett stickprov som är tillräckligt stort för att en väg ska kunna hittas från start till mål, utan att vara så stort att det tar orimligt mycket tid att hitta vägen. 
