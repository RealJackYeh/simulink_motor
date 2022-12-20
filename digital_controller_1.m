Ts = 1e-5;
tf_iq = tf(1,[1 Rs/(sigma*Ls)]);
tf_iq_z = c2d(tf_iq, Ts);