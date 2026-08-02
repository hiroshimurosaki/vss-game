# O que a IA está pensando. Existe para a TV poder mostrar ao público como o
# robô decide — é o conteúdo pedagógico da apresentação, não só depuração.
string state           # ATACAR, POSICIONAR, DEFENDER, PARADO
float64 target_x       # para onde a IA está indo
float64 target_y
float64 ball_x         # a bola COMO A IA VÊ (com atraso e ruído aplicados)
float64 ball_y
float64 linear_vel     # o que ela decidiu
float64 angular_vel
string difficulty      # FACIL, MEDIO, DIFICIL
float64 reaction_delay # segundos de atraso embutido
float64 speed_frac     # fração da velocidade máxima que ela se permite
