#!/usr/bin/env python3
"""Roteiro de ensaios para separar as causas da morte do rádio do robô.

    ./tools/painel.py                 # em outro terminal, ANTES (é ele que grava)
    ./tools/ensaio.py                 # aqui: conduz os testes e carimba os tempos
    ./tools/ensaio.py --analisar      # depois: cruza os carimbos com o log

O log do `debug_panel` diz **o que** aconteceu com precisão de milissegundo, mas
não diz **o que você estava fazendo**. Este script escreve os carimbos de início
e fim de cada ensaio no mesmo relógio (`time.time()`, mesma máquina), num arquivo
irmão em `~/.vss-game/logs/`. O `--analisar` junta os dois e devolve uma tabela
com uma linha por ensaio.

POR QUE O ROTEIRO TEM ESSA FORMA
--------------------------------
A sessão de 11/08 mostrou três coisas que restringem qualquer teste honesto:

1. **A falha trava.** Depois que o rádio morreu, ficou morto por 286 s, sem
   recuperar sozinho. Logo: um ensaio rodado com o robô já morto não mede nada,
   e um roteiro que não confere saúde entre os passos produz doze "falhou" sem
   significado nenhum. Daí o `saude()` antes e depois de cada ensaio.

2. **Duas hipóteses produzem o mesmo gráfico**: queda de tensão pelo surto dos
   motores, e conexão intermitente no 3V3/GND. Ligar a bateria é também *mexer no
   robô*, e é isso que confunde as duas. Por isso há ensaios que aplicam carga
   sem tocar no robô, e ensaios que tocam no robô sem aplicar carga.

3. **Ordem importa.** Os ensaios vão do mais brando ao mais agressivo, e o
   `travado` fica por último de propósito: ele é o mais provável de matar, e o
   que morre no fim não contamina o resto.

Os ensaios que pedem carga são conduzidos pelo próprio script, via HTTP do
painel, em vez de pelo seu teclado: acelerador exato e duração exata valem mais
que a mão humana quando o objetivo é comparar um ensaio com o outro.

Segurança: o painel para os motores sozinho 0,3 s depois que este script parar
de mandar comando. Ctrl+C a qualquer momento é seguro.

Só stdlib.
"""

import argparse
import collections
import glob
import json
import os
import sys
import threading
import time
import urllib.error
import urllib.request

LOG_DIR = os.path.expanduser('~/.vss-game/logs')
CID = 'ensaio'          # identidade fixa: o painel usa isto para contar clientes


# ── O roteiro ────────────────────────────────────────────────────────────────
#
# `carga`   = acelerador que o script aplica (None = não toca nos motores)
# `separa`  = qual hipótese este ensaio isola. É o campo mais importante:
#             ensaio que não separa nada é tempo de bancada jogado fora.

ENSAIOS = [
    # ── bloco 0: referência, sem bateria ────────────────────────────────────
    dict(id='ref-usb-parado', bloco='0 · referência', dur=30, carga=None,
         bateria=False,
         titulo='Só USB, bateria DESLIGADA, robô parado e intocado',
         separa='linha de base. Se já falhar aqui, não tem nada a ver com bateria '
                'nem com motor, e todo o resto do roteiro muda.',
         instrucao='Bateria desligada. Não encoste no robô.'),

    dict(id='ref-usb-motor', bloco='0 · referência', dur=30, carga=1.0,
         bateria=False,
         titulo='Só USB, bateria DESLIGADA, acelerador 100%',
         separa='comando de motor SEM corrente de motor. O driver não tem VM, '
                'então o PWM chaveia e as rodas quase não puxam. Falhar aqui '
                'aponta para o chaveamento/software, não para a corrente.',
         instrucao='Bateria desligada. Rodas no ar. Não encoste no robô.'),

    # ── bloco 1: bateria ligada, sem carga ──────────────────────────────────
    dict(id='bat-liga', bloco='1 · bateria sem carga', dur=45, carga=None,
         bateria=True,
         titulo='LIGAR a bateria agora, com o robô parado',
         separa='"ligar a bateria" de "puxar corrente". Se morre aqui, com motor '
                'nenhum girando, a causa é o ato de ligar/mexer — não o surto.',
         instrucao='Ligue a bateria QUANDO o ensaio começar. Depois não encoste.'),

    dict(id='bat-repouso', bloco='1 · bateria sem carga', dur=60, carga=None,
         bateria=True,
         titulo='Bateria ligada, robô parado, intocado',
         separa='se a falha precisa de estímulo ou vem sozinha com o tempo.',
         instrucao='Não encoste no robô. Só observe.'),

    # ── bloco 2: mecânico, sem carga ────────────────────────────────────────
    dict(id='mexer-fios-radio', bloco='2 · mecânico', dur=30, carga=None,
         bateria=True,
         titulo='Balançar os fios DO RÁDIO, motores parados',
         separa='conexão intermitente no módulo de rádio. Carga zero: se morrer '
                'aqui, nenhum capacitor do mundo resolve — é solda/conector.',
         instrucao='Mexa devagar em cada fio do nRF24 (VCC, GND, CE, CSN, SCK, '
                   'MOSI, MISO), um a um. Sem forçar.'),

    dict(id='mexer-fios-alim', bloco='2 · mecânico', dur=30, carga=None,
         bateria=True,
         titulo='Balançar os fios de ALIMENTAÇÃO, motores parados',
         separa='conexão intermitente na alimentação (bateria, regulador, VIN).',
         instrucao='Mexa nos fios de bateria/regulador/VIN e no conector da '
                   'bateria. Sem forçar.'),

    dict(id='mexer-robo', bloco='2 · mecânico', dur=30, carga=None,
         bateria=True,
         titulo='Levantar, inclinar e girar o robô, motores parados',
         separa='vibração/aceleração sem corrente. É o ensaio que imita "mexi no '
                'robô para ligar a bateria" sem ligar nada.',
         instrucao='Levante, incline para os dois lados, gire devagar. '
                   'Sem bater.'),

    # ── bloco 3: carga crescente, rodas no ar ───────────────────────────────
    dict(id='ar-25', bloco='3 · carga sem tração', dur=30, carga=0.25,
         bateria=True, titulo='Rodas no ar, acelerador 25%',
         separa='primeiro degrau de corrente real.',
         instrucao='Robô apoiado com as RODAS NO AR. Não encoste durante.'),

    dict(id='ar-50', bloco='3 · carga sem tração', dur=30, carga=0.5,
         bateria=True, titulo='Rodas no ar, acelerador 50%',
         separa='se existe um limiar de corrente, ele aparece entre os degraus.',
         instrucao='Rodas no ar. Não encoste durante.'),

    dict(id='ar-100', bloco='3 · carga sem tração', dur=30, carga=1.0,
         bateria=True, titulo='Rodas no ar, acelerador 100%',
         separa='corrente máxima sem carga mecânica.',
         instrucao='Rodas no ar. Não encoste durante.'),

    # ── bloco 4: carga real ─────────────────────────────────────────────────
    dict(id='chao-100', bloco='4 · carga real', dur=30, carga=1.0,
         bateria=True, titulo='No chão, acelerador 100%, andando',
         separa='condição de jogo de verdade: tração, inércia, irregularidade.',
         instrucao='Robô no chão/campo, livre para andar. Acompanhe sem tocar.'),

    dict(id='reversao', bloco='4 · carga real', dur=20, carga='reversao',
         bateria=True, titulo='Reversão brusca frente/ré a cada 0,5 s',
         separa='pico de corrente maior que o de estol contínuo — inversão com o '
                'motor ainda girando soma FCEM à tensão da ponte H.',
         instrucao='Rodas no ar. Não encoste durante.'),

    dict(id='travado', bloco='4 · carga real', dur=10, carga=1.0,
         bateria=True, titulo='RODAS TRAVADAS com a mão, acelerador 100%',
         separa='corrente de estol, o pior caso elétrico. Deliberadamente por '
                'último: é o mais provável de matar.',
         instrucao='Segure as duas rodas FIRME para não girarem. Só 10 s — '
                   'motor travado esquenta.'),

    # ── bloco 5: controle negativo de RF ────────────────────────────────────
    dict(id='distancia', bloco='5 · RF', dur=45, carga=None,
         bateria=True, titulo='Robô a 3+ metros da ponte, parado',
         separa='rádio/alcance de TUDO o mais. Se só este falha, a causa é RF e '
                'nenhuma das hipóteses elétricas se sustenta.',
         instrucao='Afaste o robô o máximo que o cabo USB permitir (ou use '
                   'extensão). Motores parados.'),
]

# Percussão: localizar o ponto do contato marginal.
#
# Nasceu do resultado de 11/08: balançar os FIOS foi limpo (100% de ACK) e
# balançar o ROBÔ matou (28,9%), com corrente de motor zero nos dois. Ou seja, o
# contato não está nas pontas dupont — está em algo que responde a inércia. Os
# ensaios acima provaram que existe; estes dizem ONDE.
#
# Regra de ouro: um ponto por ensaio. Dois pontos por ensaio e o resultado não
# aponta para nenhum dos dois.
PERCUSSAO = [
    dict(id='perc-controle', bloco='7 · percussão', dur=20, carga=None, bateria=True,
         titulo='Controle: robô parado, ninguém encosta',
         separa='a linha de base desta bateria de testes. Se cair aqui, os '
                'ensaios seguintes não significam nada.',
         instrucao='Bateria ligada, robô parado na bancada. NÃO encoste.'),

    dict(id='perc-bancada', bloco='7 · percussão', dur=20, carga=None, bateria=True,
         titulo='Bater de leve NA BANCADA ao lado do robô',
         separa='inércia pura: choque sem tocar no robô e sem puxar cabo nenhum. '
                'Se isto derruba, é contato marginal — não é cabo, não é mão.',
         instrucao='Bata com o dedo na bancada ao lado, a cada 2 s. Não encoste '
                   'no robô nem nos cabos.'),

    dict(id='perc-modulo', bloco='7 · percussão', dur=20, carga=None, bateria=True,
         titulo='Pressionar o MÓDULO nRF24 contra o header',
         separa='assentamento do módulo no conector.',
         instrucao='Pressione o módulo para baixo, firme, e solte. Repita a cada '
                   '3 s. Só o módulo.'),

    dict(id='perc-solda-modulo', bloco='7 · percussão', dur=20, carga=None,
         bateria=True, titulo='Apertar os PINOS do módulo, um a um',
         separa='solda fria no header do próprio módulo — o suspeito nº 1 quando '
                'balançar os fios não reproduz mas balançar o robô sim.',
         instrucao='Com um palito/bastão plástico, pressione cada pino do header '
                   'do rádio, um de cada vez, 2 s em cada.'),

    dict(id='perc-nano', bloco='7 · percussão', dur=20, carga=None, bateria=True,
         titulo='Pressionar o NANO no soquete',
         separa='Nano mal assentado no soquete da placa.',
         instrucao='Pressione o Nano para baixo e solte, a cada 3 s.'),

    dict(id='perc-bateria', bloco='7 · percussão', dur=20, carga=None, bateria=True,
         titulo='Mexer a BATERIA no suporte',
         separa='contato do suporte/conector da bateria. Massa grande = a que '
                'mais se move quando o robô balança.',
         instrucao='Empurre a bateria de leve no suporte, para os lados e para '
                   'baixo. Sem desconectar.'),

    dict(id='perc-torcao', bloco='7 · percussão', dur=20, carga=None, bateria=True,
         titulo='Torcer levemente o chassi/placa',
         separa='trinca de solda que só abre sob flexão da placa.',
         instrucao='Segure o chassi pelas pontas e torça MUITO de leve, nos dois '
                   'sentidos, a cada 3 s.'),
]

# Ensaios de recuperação: só fazem sentido DEPOIS que o robô morreu. O que
# ressuscita o rádio diz onde ele estava travado.
RECUPERACAO = [
    dict(id='rec-nada', bloco='6 · recuperação', dur=20, carga=None, bateria=True,
         titulo='Não fazer nada por 20 s',
         separa='se o rádio volta sozinho. Em 11/08 não voltou em 286 s.',
         instrucao='Não faça nada. Só espere.'),

    dict(id='rec-reset-nano', bloco='6 · recuperação', dur=20, carga=None,
         bateria=True, titulo='Apertar o botão RESET do Nano',
         separa='estado travado no FIRMWARE (volta) de estado travado no CHIP do '
                'rádio (não volta, porque o reset do Nano não corta o 3V3 dele).',
         instrucao='Aperte o RESET do Nano uma vez, quando começar.'),

    dict(id='rec-power-radio', bloco='6 · recuperação', dur=20, carga=None,
         bateria=True, titulo='Cortar e religar a alimentação SÓ do rádio',
         separa='confirma travamento do nRF24: se só isto ressuscita, o chip '
                'estava latchado e o remédio é alimentação, não código.',
         instrucao='Desconecte o VCC do módulo de rádio, conte até 3, reconecte. '
                   'Depois aperte o RESET do Nano.'),
]


# ── Conversa com o painel ────────────────────────────────────────────────────

def painel(url, left=0.0, right=0.0, robot_id=1, timeout=3.0):
    """Manda um comando e devolve o estado. left=right=0 é uma leitura inócua."""
    body = json.dumps({'cid': CID, 'id': robot_id,
                       'left': left, 'right': right}).encode()
    req = urllib.request.Request(url.rstrip('/') + '/cmd', data=body,
                                 headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read())


def saude(url, robot_id):
    """Resume o estado em uma palavra. É o portão entre um ensaio e o próximo.

    O sinal primário é o **ACK**, não a serial do robô. O ACK é respondido pelo
    hardware do nRF24 e chega pela ponte, então continua existindo com o robô
    fora do USB — que é exatamente a condição dos ensaios em que ele anda solto.
    Julgar por `rx_ok` (serial do robô) fez a primeira rodada marcar seis
    ensaios como INVÁLIDO quando os seis tinham passado com 86–100% de ACK.

    A serial do robô continua sendo lida, mas como detalhe: ela diz *onde* o
    pacote morreu, e só está disponível quando o robô está no cabo.
    """
    try:
        st = painel(url, robot_id=robot_id)
    except (urllib.error.URLError, OSError) as exc:
        return 'SEM PAINEL', {'erro': repr(exc)}
    n = st['numbers']
    ack, noack = n.get('ack') or 0, n.get('noack') or 0
    ok, chk, sb = n.get('rx_ok') or 0, n.get('rx_checksum') or 0, n.get('rx_startbyte') or 0
    total = ack + noack
    pct = 100.0 * ack / total if total else None

    if pct is None:
        estado = 'SEM PONTE'          # a ponte não fala: nem dá para julgar
    elif pct >= 95:
        estado = 'VIVO'
    elif pct >= 20:
        estado = 'DEGRADADO'
    else:
        estado = 'MORTO'

    det = {'ack%': round(pct, 1) if pct is not None else None,
           'clientes': n.get('clients'), 'log_file': n.get('log_file')}
    det['robo_no_usb'] = (ok + chk + sb) > 0
    if det['robo_no_usb']:
        det.update(rx_ok=round(ok, 1), checksum=round(chk, 1), startbyte=round(sb, 1))
    return estado, det


class Motor(threading.Thread):
    """Segura o acelerador durante o ensaio.

    Tem de ser um fluxo contínuo: o painel zera os motores 0,3 s depois do
    último comando. Esse watchdog é o que torna o Ctrl+C seguro.
    """

    def __init__(self, url, robot_id, carga):
        super().__init__(daemon=True)
        self.url, self.robot_id, self.carga = url, robot_id, carga
        self.running = True
        self.erros = 0

    def run(self):
        t0 = time.time()
        while self.running:
            if self.carga == 'reversao':
                v = 1.0 if int((time.time() - t0) / 0.5) % 2 == 0 else -1.0
            else:
                v = float(self.carga)
            try:
                painel(self.url, left=v, right=v, robot_id=self.robot_id)
            except (urllib.error.URLError, OSError):
                self.erros += 1
            time.sleep(0.1)


# ── Carimbos ─────────────────────────────────────────────────────────────────

class Marcador:
    def __init__(self, path):
        self.fh = open(path, 'a', buffering=1)
        self.path = path

    def marca(self, **kw):
        rec = dict(kw)
        rec['t'] = round(time.time(), 3)
        rec['h'] = time.strftime('%H:%M:%S')
        self.fh.write(json.dumps(rec, ensure_ascii=False) + '\n')
        return rec


# ── Condução ─────────────────────────────────────────────────────────────────

def conduzir(ensaio, marc, url, robot_id):
    """Roda um ensaio. Devolve False se o operador mandou parar."""
    print()
    print('═' * 72)
    print(f'  {ensaio["bloco"]}   ·   {ensaio["id"]}')
    print(f'  {ensaio["titulo"]}')
    print('─' * 72)
    print(f'  o que separa : {ensaio["separa"]}')
    print(f'  o que fazer  : {ensaio["instrucao"]}')
    carga = ensaio['carga']
    print(f'  duração      : {ensaio["dur"]} s'
          + (f'   ·   acelerador: {carga}' if carga is not None else '   ·   sem comando de motor'))
    print('═' * 72)

    antes, det = saude(url, robot_id)
    print(f'  saúde ANTES: {antes}  {det}')
    if antes == 'SEM PAINEL':
        print('\n  O painel não respondeu. Suba `./tools/painel.py` antes — sem ele')
        print('  nada é gravado e o ensaio não serve para nada.')
        return False
    while antes != 'VIVO':
        print('\n  !! O robô NÃO está saudável antes de começar.')
        print('     Um ensaio que começa morto não mede nada — a falha trava e')
        print('     contamina tudo que vier depois. Ressuscite (reset do Nano,')
        print('     e se preciso religue o VCC do rádio) e só então siga.')
        r = input('     [r] reconferir · [f] forçar assim mesmo · [s] sair: ').strip().lower()
        if r == 's':
            return False
        if r == 'f':
            break
        antes, det = saude(url, robot_id)
        print(f'  saúde ANTES: {antes}  {det}')

    resp = input('\n  ENTER para começar · [p] pular · [s] sair do roteiro: ').strip().lower()
    if resp == 's':
        return False
    if resp == 'p':
        marc.marca(k='marca', ensaio=ensaio['id'], fase='pulado')
        return True

    marc.marca(k='marca', ensaio=ensaio['id'], fase='inicio', bloco=ensaio['bloco'],
               titulo=ensaio['titulo'], separa=ensaio['separa'],
               carga=carga, dur=ensaio['dur'], saude_antes=antes, det_antes=det)

    motor = None
    if carga is not None:
        motor = Motor(url, robot_id, carga)
        motor.start()

    try:
        for restante in range(ensaio['dur'], 0, -1):
            print(f'\r  rodando... {restante:3d} s   (Ctrl+C interrompe)', end='', flush=True)
            time.sleep(1)
        print('\r' + ' ' * 60, end='\r')
    except KeyboardInterrupt:
        print('\n  interrompido pelo operador')
    finally:
        if motor:
            motor.running = False
            motor.join(timeout=1)

    depois, det2 = saude(url, robot_id)
    marc.marca(k='marca', ensaio=ensaio['id'], fase='fim',
               saude_depois=depois, det_depois=det2)
    print(f'  saúde DEPOIS: {depois}  {det2}')
    if antes == 'VIVO' and depois != 'VIVO':
        print('  >>> ESTE ENSAIO MATOU O ROBÔ. É o achado mais valioso do roteiro.')

    obs = input('  observação (o que você viu/ouviu, ENTER para pular): ').strip()
    if obs:
        marc.marca(k='marca', ensaio=ensaio['id'], fase='obs', texto=obs)
    return True


# ── Análise ──────────────────────────────────────────────────────────────────

def carregar(padrao):
    recs = []
    for path in sorted(glob.glob(os.path.join(LOG_DIR, padrao))):
        for linha in open(path):
            try:
                recs.append(json.loads(linha))
            except json.JSONDecodeError:
                pass
    return recs


def analisar(ensaio_file=None):
    """Cruza os carimbos com o log do painel e imprime uma linha por ensaio."""
    marcas = carregar(os.path.basename(ensaio_file) if ensaio_file else 'ensaio-*.jsonl')
    painel_recs = [r for r in carregar('debug_panel-*.jsonl') if r.get('k') == 'raw']
    if not marcas:
        print('nenhum carimbo em', LOG_DIR); return 1
    if not painel_recs:
        print('nenhum log do painel em', LOG_DIR); return 1
    painel_recs.sort(key=lambda r: r['t'])

    # Emparelha inicio/fim por ensaio, na ordem em que aconteceram.
    abertos, janelas = {}, []
    for m in marcas:
        # `k` filtra os registros de sessão, que também têm fase='fim' e
        # derrubavam o emparelhamento por não terem `ensaio`.
        if m.get('k') != 'marca' or 'ensaio' not in m:
            continue
        if m.get('fase') == 'inicio':
            abertos[m['ensaio']] = m
        elif m.get('fase') == 'fim' and m['ensaio'] in abertos:
            janelas.append((abertos.pop(m['ensaio']), m))
    for ini in abertos.values():                     # ensaio sem fim: Ctrl+C
        janelas.append((ini, None))

    print(f'\n{len(janelas)} ensaio(s) · log do painel com {len(painel_recs)} linhas\n')
    cab = (f'{"ensaio":<18}{"carga":>7}{"dur":>6}{"OK/s":>7}{"CHK/s":>7}'
           f'{"SB/s":>7}{"boot":>6}{"ACK%":>7}  {"perfil":<14}veredito')
    print(cab); print('─' * len(cab))

    for ini, fim in janelas:
        t0 = ini['t']
        t1 = fim['t'] if fim else t0 + ini.get('dur', 0)
        dur = max(t1 - t0, 0.001)
        jan = [r for r in painel_recs if t0 <= r['t'] <= t1]
        c = collections.Counter()
        for r in jan:
            l = r['line']
            if r['src'] == 'robo':
                if l.startswith('OK |'): c['ok'] += 1
                elif l.startswith('CHECKSUM FAIL'): c['chk'] += 1
                elif l.startswith('START BYTE'): c['sb'] += 1
                elif l.startswith(('robot_rx', 'radio OK')): c['boot'] += 1
            else:
                if l.endswith('SEM ACK'): c['sem'] += 1
                elif l.endswith('ACK'): c['ack'] += 1
        ack_tot = c['ack'] + c['sem']
        ackpct = 100.0 * c['ack'] / ack_tot if ack_tot else float('nan')

        # O veredito sai do ACK medido no log, não da saúde carimbada: o ACK vem
        # da ponte e vale também com o robô fora do USB. Comparar o começo com o
        # fim da janela é o que distingue "já estava ruim" de "este ensaio matou".
        def pct(a, b):
            w = [r for r in jan if r['src'] == 'ponte' and a <= r['t'] - t0 <= b]
            return (100.0 * sum(1 for r in w if not r['line'].endswith('SEM ACK'))
                    / len(w)) if w else None
        ini_pct, fim_pct = pct(0, min(5, dur / 3)), pct(max(0, dur - 5), dur)

        # Perfil por blocos de 5 s. Uma média esconde a forma, e é a forma que
        # diagnostica: `▒█████▒░░` (cai, recupera, morre depois) é contato
        # intermitente; `██░░░░` (cai e fica) é outra coisa; `████` é saúde.
        perfil = ''
        for b in range(max(1, int(dur // 5))):
            p = pct(b * 5, (b + 1) * 5)
            perfil += ' ' if p is None else '█' if p > 95 else '▓' if p > 50 else '▒' if p > 5 else '░'

        if ack_tot == 0:
            vered = 'SEM DADOS — a ponte não falou nesta janela'
        elif ini_pct is not None and ini_pct < 50:
            vered = f'INVÁLIDO — começou em {ini_pct:.0f}% de ACK'
        elif fim_pct is not None and fim_pct < 50:
            vered = f'*** MATOU O ROBÔ ({ini_pct:.0f}% → {fim_pct:.0f}% de ACK) ***'
        elif ackpct < 99 or c['boot'] or c['chk'] or c['sb']:
            vered = (f'sobreviveu, mas sujou ({c["boot"]//2} reboot, {c["chk"]} chk, '
                     f'{c["sb"]} sb)')
        else:
            vered = 'limpo'
        if not (c['ok'] or c['chk'] or c['sb'] or c['boot']):
            vered += '  [robô fora do USB]'

        carga = ini.get('carga')
        carga_s = '—' if carga is None else str(carga)
        print(f'{ini["ensaio"]:<18}{carga_s:>7}{dur:>5.0f}s{c["ok"]/dur:>7.1f}'
              f'{c["chk"]/dur:>7.1f}{c["sb"]/dur:>7.1f}{c["boot"]//2:>6}'
              f'{ackpct:>7.1f}  {perfil:<14}{vered}')

    obs = [m for m in marcas if m.get('fase') == 'obs']
    if obs:
        print('\nobservações do operador:')
        for m in obs:
            print(f'  {m["h"]}  {m["ensaio"]:<18} {m["texto"]}')
    print(f'\nleia junto: {LOG_DIR}')
    return 0


# ── main ─────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--painel', default='http://localhost:8061',
                    help='URL do debug_panel (default: %(default)s)')
    ap.add_argument('--id', type=int, default=1, help='robot_id (default: 1)')
    ap.add_argument('--bloco', help='rodar só os ensaios cujo bloco começa assim (ex.: 3)')
    ap.add_argument('--recuperacao', action='store_true',
                    help='rodar o bloco 6, para depois que o robô já morreu')
    ap.add_argument('--percussao', action='store_true',
                    help='rodar o bloco 7: localiza o ponto do contato marginal')
    ap.add_argument('--analisar', nargs='?', const='', metavar='ARQUIVO',
                    help='não roda nada: cruza os carimbos com o log e sai')
    args = ap.parse_args()

    if args.analisar is not None:
        return analisar(args.analisar or None)

    roteiro = (RECUPERACAO if args.recuperacao else
               PERCUSSAO if args.percussao else ENSAIOS)
    if args.bloco:
        roteiro = [e for e in roteiro if e['bloco'].startswith(args.bloco)]
    if not roteiro:
        print('nenhum ensaio bate com esse filtro'); return 1

    estado, det = saude(args.painel, args.id)
    if estado == 'SEM PAINEL':
        print(f'O painel não respondeu em {args.painel}.')
        print('Suba `./tools/painel.py` em outro terminal primeiro — é ele que')
        print('grava o log; sem ele estes carimbos não têm com o que cruzar.')
        return 1
    if (det.get('clientes') or 0) > 1:
        print('AVISO: o painel vê mais de um cliente. Feche a aba do navegador —')
        print('       dois comandos alternados viram "pacote fantasma" no log.')

    os.makedirs(LOG_DIR, exist_ok=True)
    path = os.path.join(LOG_DIR, 'ensaio-' + time.strftime('%Y%m%d-%H%M%S') + '.jsonl')
    marc = Marcador(path)
    marc.marca(k='sessao', inicio=time.strftime('%Y-%m-%d %H:%M:%S'),
               painel=args.painel, robot_id=args.id, argv=sys.argv,
               log_painel=det.get('log_file'), roteiro=[e['id'] for e in roteiro])

    print(f'\ncarimbos em  : {path}')
    print(f'log do painel: {det.get("log_file")}')
    print(f'\n{len(roteiro)} ensaios. Entre um e outro dá para parar; o que já')
    print('rodou continua valendo. Ctrl+C para os motores em 0,3 s.')

    try:
        for e in roteiro:
            if not conduzir(e, marc, args.painel, args.id):
                break
    except KeyboardInterrupt:
        print('\nroteiro interrompido')
    finally:
        marc.marca(k='sessao', fase='fim')
        print(f'\npronto. Agora rode:\n\n    ./tools/ensaio.py --analisar\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
