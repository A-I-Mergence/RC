Regulateur RC:
Ce type de régulateur permet de faire en sorte que notre moteur atteignent une certaine vitesse
(appelé Setpoint) le plus rapidement possible et sans trop de dépassement.

Cette librairie prend en entrée la valeur setpoint (vitesse à atteindre par notre moteur)
ainsi que Input qui prend comme valeur la vitesse réelle du moteur.
la valeur de sortie Output donne la valeur en pwm à passer au moteur. 

Fonctionnement du régulataur RC:

Une précommande "PreCommande[k]" commence par lisser la valeur de notre commande (setpoint)
on calcul ensuite l'erreur entre la précommande et la valeur réelle en sortie de notre moteur.
Le Reg_RC[k] et Reg_1s[k] sont ensuite appliqué à l'erreur et pour Reg_RC et à la sortie de 
Reg_RC pour Reg_1s. 
Comme chacun des calculs prennent en compte la valeur à t-1, nous mettons ensuite à jour les 
les différentes variables.
La dernière étape de ce régutaleur est le filtrage des données.
Nous application donc un filtre à la valeur en sortie de Reg_1safin que les valeurs de pwm 
données au moteur ne dépassent pas les valeurs de -1 et de 1.