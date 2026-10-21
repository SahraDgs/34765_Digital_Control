Gs = tf([1], [1 1]);
Gz1 = c2d(Gs, 0.1, 'impulse');
Gz2 = c2d(Gs, 0.1, 'zoh');

stepplot(Gs, Gz1, Gz2)

Gs_bis = tf([1], [1 0 0]);
Gz1_bis = c2d(Gs_bis, 1, 'zoh');
Gz1_bis