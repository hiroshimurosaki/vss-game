# Tudo que a TV e o operador precisam saber. Uma mensagem só, publicada sempre:
# a tela é burra de propósito, ela só desenha o que chega aqui.

string IDLE=IDLE                 # tela de atração, sem jogador
string REGISTRO=REGISTRO         # digitando o nome
string CONTAGEM=CONTAGEM         # 3, 2, 1...
string JOGANDO=JOGANDO
string GOL=GOL                   # comemoração, robôs voltando ao lugar
string FIM=FIM                   # resultado e ranking
string PAUSA=PAUSA

string state
string player_name

uint8 player_score
uint8 ai_score
uint8 target_score               # quantos gols ganham a partida

float64 elapsed                  # cronômetro, em segundos
float64 time_limit
float64 state_remaining          # quanto falta do estado atual (contagem, gol)

bool player_won
bool ranked                      # entrou no top 10
uint8 rank_position              # 0 = não entrou

string last_scorer               # JOGADOR ou IA
string difficulty
