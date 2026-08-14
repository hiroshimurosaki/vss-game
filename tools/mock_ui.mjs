#!/usr/bin/env node
/* ═══════════════════════════════════════════════════════════════════════════
   mock_ui.mjs — FERRAMENTA DE BANCADA. Não faz parte do jogo.
   ═══════════════════════════════════════════════════════════════════════════

   Existe por um motivo só: abrir as telas web do árbitro no navegador, num
   PC sem Python e sem ROS, para conferir o VISUAL. Nada aqui conhece as
   regras do jogo — não há máquina de estados, não há placar que evolui, não
   há cronômetro que anda. Os snapshots são fixos e escritos à mão, um por
   cena, e repetidos a 10 Hz. Se o layout de uma tela mudar e a cena parecer
   errada, a cena é que está velha; corrija a constante aqui embaixo.

   Zero dependências externas: só http, fs, path, crypto e url. O WebSocket é
   implementado à mão (frame simples de texto), o bastante para as telas.

   ── O truque da cena ───────────────────────────────────────────────────────
   As páginas se conectam com `new WebSocket(\`ws://${location.host}/ws\`)` —
   sem repassar a querystring da própria página. Ou seja: abrir
   `/x1?cena=fim` NÃO faz o `?cena=fim` chegar ao `/ws` sozinho.

   Como não podemos tocar no HTML (é a tela de verdade), o servidor resolve
   isso do lado dele: no handshake do WebSocket, quando a query do `/ws` não
   traz `cena`, lemos o header `Referer` — que o navegador preenche com a URL
   da PÁGINA que abriu a conexão, querystring inclusa — e tiramos a cena de
   lá. É por isso que `http://localhost:8090/x1?cena=fim` funciona.

   Precedência: `?cena=` no /ws  >  cena do Referer  >  `--cena=` da linha de
   comando  >  `idle`. Como a resolução é por CONEXÃO, dá para deixar várias
   abas abertas, cada uma numa cena diferente, ao mesmo tempo.

   ── Uso ────────────────────────────────────────────────────────────────────
       node tools/mock_ui.mjs
       node tools/mock_ui.mjs --porta=9000 --cena=jogando

   ── Symlinks ───────────────────────────────────────────────────────────────
   Neste checkout Windows, `src/game_master/web/vss.css` e
   `src/game_master/web/fonts` são symlinks salvos como TEXTO (o arquivo
   contém literalmente `../../../web/vss.css`). Servir esses caminhos
   entregaria a string, não o CSS. Por isso as rotas de estilo e fonte
   apontam direto para `web/vss.css` e `web/fonts/` na raiz do repositório.
   ═══════════════════════════════════════════════════════════════════════════ */

'use strict';

import http from 'node:http';
import fs from 'node:fs';
import path from 'node:path';
import crypto from 'node:crypto';
import { fileURLToPath } from 'node:url';

/* ── Caminhos ──────────────────────────────────────────────────────────────
   Resolvidos a partir do próprio script para o servidor subir de qualquer
   diretório. O script mora em <raiz>/tools/, logo a raiz é dois níveis acima. */
const AQUI = path.dirname(fileURLToPath(import.meta.url));
const RAIZ = path.resolve(AQUI, '..');
const DIR_TELAS = path.join(RAIZ, 'src', 'game_master', 'web');
const DIR_FONTES = path.join(RAIZ, 'web', 'fonts');
const ARQ_CSS = path.join(RAIZ, 'web', 'vss.css');

/* ── Linha de comando ───────────────────────────────────────────────────── */
function arg(nome, padrao) {
  const pref = `--${nome}=`;
  const achou = process.argv.slice(2).find((a) => a.startsWith(pref));
  return achou ? achou.slice(pref.length) : padrao;
}

const PORTA = Number(arg('porta', '8090'));
const CENA_CLI = arg('cena', 'idle');
const HZ = 10; // o árbitro de verdade manda a 30 Hz; 10 basta para o print

/* ═══════════════════════════════════════════════════════════════════════════
   AS CENAS
   Cada cena é um snapshot inteiro, fixo, no formato do modo X1. Nenhum campo
   inventado: o contrato é o do árbitro.
   ═══════════════════════════════════════════════════════════════════════════ */

/* O ranking de fundo: 10 partidas plausíveis, ordenadas pelo tempo de quem
   venceu, crescente. É o que a tela IDLE mostra rolando. */
const RANKING = [
  { name_a: 'RAFAELA',  name_b: 'DIOGO',    score_a: 2, score_b: 0, time_a: 12.4, time_b: 19.8, winner: 'A', date: '2026-08-14' },
  { name_a: 'BRUNO',    name_b: 'TAINA',    score_a: 2, score_b: 1, time_a: 15.9, time_b: 24.1, winner: 'A', date: '2026-08-14' },
  { name_a: 'CAIO',     name_b: 'LARISSA',  score_a: 2, score_b: 0, time_a: 21.6, time_b: 33.0, winner: 'A', date: '2026-08-14' },
  { name_a: 'MATEUS',   name_b: 'PRISCILA', score_a: 1, score_b: 2, time_a: 38.7, time_b: 25.3, winner: 'B', date: '2026-08-14' },
  { name_a: 'JULIANA',  name_b: 'OTAVIO',   score_a: 2, score_b: 0, time_a: 28.9, time_b: 41.2, winner: 'A', date: '2026-08-14' },
  { name_a: 'GUSTAVO',  name_b: 'BEATRIZ',  score_a: 1, score_b: 2, time_a: 47.0, time_b: 32.5, winner: 'B', date: '2026-08-14' },
  { name_a: 'LEANDRO',  name_b: 'CAMILA',   score_a: 2, score_b: 0, time_a: 36.2, time_b: 50.4, winner: 'A', date: '2026-08-14' },
  { name_a: 'VITORIA',  name_b: 'RODRIGO',  score_a: 2, score_b: 1, time_a: 41.7, time_b: 55.9, winner: 'A', date: '2026-08-14' },
  { name_a: 'THIAGO',   name_b: 'AMANDA',   score_a: 1, score_b: 2, time_a: 58.6, time_b: 48.3, winner: 'B', date: '2026-08-14' },
  { name_a: 'PATRICIA', name_b: 'EDUARDO',  score_a: 2, score_b: 1, time_a: 55.1, time_b: 59.4, winner: 'A', date: '2026-08-14' },
];

/* A partida da cena `fim`, já como registro de ranking. Entra em 3º lugar
   (tempo 18.3, entre BRUNO 15.9 e CAIO 21.6) e empurra o último para fora —
   o quadro guarda 10. */
const PARTIDA_FIM = {
  name_a: 'FERNANDO', name_b: 'MARIANA',
  score_a: 2, score_b: 1, time_a: 18.3, time_b: 31.2,
  winner: 'A', date: '2026-08-14',
};
const RANKING_COM_FIM = [
  ...RANKING.slice(0, 2), PARTIDA_FIM, ...RANKING.slice(2),
].slice(0, 10);

const ROUND_1 = { number: 1, time: 18.3, winner: 'A' };
const ROUND_2 = { number: 2, time: 31.2, winner: 'B' };
const ROUND_3 = { number: 3, time: 22.5, winner: 'A' };

/* Molde comum. Cada cena sobrescreve o que lhe interessa; assim nenhum campo
   do contrato fica faltando por esquecimento. */
function snapshot(x1, topo) {
  return {
    mode: 'x1',
    state: 'IDLE',
    player_name: '',
    player_score: 0,
    ai_score: 0,
    target_score: 2,
    elapsed: 0.0,
    time_limit: 90.0,
    state_remaining: 0.0,
    player_won: false,
    ranked: false,
    rank_position: 0,
    last_scorer: '',
    difficulty: 'MEDIO',
    scores: RANKING,
    ...topo,
    x1: {
      name_a: '',
      name_b: '',
      score_a: 0,
      score_b: 0,
      round_number: 0,
      rounds_to_win: 2,
      max_rounds: 3,
      round_limit: 90.0,
      // `null`, e não `0.0`: é o que o árbitro manda para quem ainda não
      // venceu nenhum round (`_num` transforma o NaN em None). Zerar aqui
      // esconderia justamente o caminho que a tela precisa acertar — ela tem
      // que escrever "—", e com 0.0 ela escreveria "0:00.0" sem ninguém notar.
      best_a: null,
      best_b: null,
      total_a: 0.0,
      total_b: 0.0,
      winner: '',
      rounds: [],
      ...x1,
    },
  };
}

/* O bloco x1 da partida em andamento — FERNANDO 1 x 0 MARIANA no round 2,
   com o round 1 já fechado. Reaproveitado por jogando/jogando_urgente/pausa. */
const X1_JOGANDO = {
  name_a: 'FERNANDO', name_b: 'MARIANA',
  score_a: 1, score_b: 0,
  round_number: 2,
  best_a: 18.3, best_b: null,
  total_a: 18.3, total_b: 0.0,
  rounds: [ROUND_1],
};

const TOPO_JOGANDO = {
  player_name: 'FERNANDO',
  player_score: 1,
  ai_score: 0,
};

const CENAS = {
  idle: snapshot({}, { state: 'IDLE' }),

  registro: snapshot({ round_number: 0 }, { state: 'REGISTRO' }),

  contagem: snapshot(
    {
      name_a: 'FERNANDO', name_b: 'MARIANA',
      round_number: 1,
    },
    {
      state: 'CONTAGEM',
      state_remaining: 2.0,
      player_name: 'FERNANDO',
    },
  ),

  jogando: snapshot(X1_JOGANDO, {
    ...TOPO_JOGANDO,
    state: 'JOGANDO',
    elapsed: 24.7,
    // O que sobra do teto de 90 s. As telas preferem este número à conta
    // local, então deixá-lo em zero acendia o alerta laranja no meio do round.
    state_remaining: 65.3,
  }),

  /* Mesma partida, só que perto do teto de 90 s: é aqui que se confere o
     cronômetro laranja. */
  jogando_urgente: snapshot(X1_JOGANDO, {
    ...TOPO_JOGANDO,
    state: 'JOGANDO',
    elapsed: 78.0,
    state_remaining: 12.0,
  }),

  gol: snapshot(
    {
      name_a: 'FERNANDO', name_b: 'MARIANA',
      score_a: 1, score_b: 1,
      round_number: 2,
      best_a: 18.3, best_b: 31.2,
      total_a: 18.3, total_b: 31.2,
      rounds: [ROUND_1, ROUND_2],
    },
    {
      state: 'GOL',
      player_name: 'FERNANDO',
      player_score: 1,
      ai_score: 1,
      elapsed: 31.2,
      last_scorer: 'B',
    },
  ),

  /* Intervalo entre rounds: a trilha já tem os dois fechados e o round_number
     aponta para o PRÓXIMO (3), que é o que a tela anuncia. */
  round: snapshot(
    {
      name_a: 'FERNANDO', name_b: 'MARIANA',
      score_a: 1, score_b: 1,
      round_number: 3,
      best_a: 18.3, best_b: 31.2,
      total_a: 18.3, total_b: 31.2,
      rounds: [ROUND_1, ROUND_2],
    },
    {
      state: 'ROUND',
      player_name: 'FERNANDO',
      player_score: 1,
      ai_score: 1,
      elapsed: 0.0,
      state_remaining: 3.0,
      last_scorer: 'B',
    },
  ),

  fim: snapshot(
    {
      name_a: 'FERNANDO', name_b: 'MARIANA',
      score_a: 2, score_b: 1,
      round_number: 3,
      best_a: 18.3, best_b: 31.2,
      total_a: 40.8, total_b: 31.2,
      winner: 'A',
      rounds: [ROUND_1, ROUND_2, ROUND_3],
    },
    {
      state: 'FIM',
      player_name: 'FERNANDO',
      player_score: 2,
      ai_score: 1,
      elapsed: 22.5,
      ranked: true,
      rank_position: 3,
      last_scorer: 'A',
      scores: RANKING_COM_FIM,
    },
  ),

  pausa: snapshot(X1_JOGANDO, {
    ...TOPO_JOGANDO,
    state: 'PAUSA',
    elapsed: 24.7,
    state_remaining: 65.3,
  }),
};

const NOMES_CENAS = Object.keys(CENAS);
const CENA_PADRAO = CENAS[CENA_CLI] ? CENA_CLI : 'idle';
if (!CENAS[CENA_CLI]) {
  console.error(`[mock_ui] cena "${CENA_CLI}" não existe; usando "idle".`);
  console.error(`[mock_ui] cenas: ${NOMES_CENAS.join(', ')}`);
}

/* ═══════════════════════════════════════════════════════════════════════════
   ARQUIVOS
   ═══════════════════════════════════════════════════════════════════════════ */

const TELAS = {
  '/': 'x1.html',
  '/x1': 'x1.html',
  '/tv': 'tv.html',
  '/duelo': 'duelo.html',
  '/operador': 'operator.html',
};

function responde(res, codigo, tipo, corpo) {
  res.writeHead(codigo, {
    'Content-Type': tipo,
    'Content-Length': Buffer.byteLength(corpo),
    'Cache-Control': 'no-store',
  });
  res.end(corpo);
}

/* Só serve o que estiver realmente dentro do diretório permitido: normaliza e
   confere o prefixo antes de abrir. Evita `..` na URL virar leitura de disco. */
function dentro(base, alvo) {
  const rel = path.relative(base, alvo);
  return rel !== '' && !rel.startsWith('..') && !path.isAbsolute(rel);
}

function serveArquivo(res, arquivo, tipo, dica) {
  let dados;
  try {
    dados = fs.readFileSync(arquivo);
  } catch {
    responde(res, 404, 'text/plain; charset=utf-8',
      `404 — arquivo não encontrado:\n  ${arquivo}\n\n${dica || ''}\n`);
    return;
  }
  res.writeHead(200, {
    'Content-Type': tipo,
    'Content-Length': dados.length,
    'Cache-Control': 'no-store',
  });
  res.end(dados);
}

const servidor = http.createServer((req, res) => {
  const url = new URL(req.url, `http://${req.headers.host || 'localhost'}`);
  const rota = url.pathname.replace(/\/+$/, '') || '/';

  if (TELAS[rota]) {
    const arquivo = path.join(DIR_TELAS, TELAS[rota]);
    serveArquivo(res, arquivo, 'text/html; charset=utf-8',
      `A tela "${TELAS[rota]}" ainda não existe neste checkout.\n` +
      `Enquanto isso, use /duelo ou /operador.`);
    return;
  }

  if (rota === '/vss.css') {
    // Direto na raiz: src/game_master/web/vss.css é symlink-texto neste checkout.
    serveArquivo(res, ARQ_CSS, 'text/css; charset=utf-8');
    return;
  }

  const fonte = /^\/fonts\/([A-Za-z0-9._-]+\.woff2)$/.exec(rota);
  if (fonte) {
    const arquivo = path.join(DIR_FONTES, fonte[1]);
    if (!dentro(DIR_FONTES, arquivo)) {
      responde(res, 403, 'text/plain; charset=utf-8', '403 — fora do diretório de fontes\n');
      return;
    }
    serveArquivo(res, arquivo, 'font/woff2');
    return;
  }

  responde(res, 404, 'text/plain; charset=utf-8',
    `404 — ${rota}\n\nRotas: ${Object.keys(TELAS).join(', ')}, /vss.css, /fonts/<nome>.woff2, /ws\n`);
});

/* ═══════════════════════════════════════════════════════════════════════════
   WEBSOCKET À MÃO (RFC 6455, o pedacinho que as telas usam)

   Servidor→cliente: só frames de texto NÃO mascarados, FIN=1. O snapshot passa
   de 125 bytes, então o comprimento estendido de 16 bits é obrigatório.
   Cliente→servidor: o navegador SEMPRE mascara; desmascaramos o suficiente
   para logar o comando no terminal, responder ping e fechar limpo.
   ═══════════════════════════════════════════════════════════════════════════ */

const GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11';

function frame(opcode, payload) {
  const dados = Buffer.isBuffer(payload) ? payload : Buffer.from(payload, 'utf8');
  const n = dados.length;
  let cab;
  if (n < 126) {
    cab = Buffer.alloc(2);
    cab[1] = n;
  } else if (n < 65536) {
    cab = Buffer.alloc(4);
    cab[1] = 126;
    cab.writeUInt16BE(n, 2);
  } else {
    cab = Buffer.alloc(10);
    cab[1] = 127;
    cab.writeBigUInt64BE(BigInt(n), 2);
  }
  cab[0] = 0x80 | opcode; // FIN + opcode; sem máscara (servidor não mascara)
  return Buffer.concat([cab, dados]);
}

const clientes = new Set();

servidor.on('upgrade', (req, socket) => {
  const url = new URL(req.url, `http://${req.headers.host || 'localhost'}`);
  if (url.pathname !== '/ws') {
    socket.end('HTTP/1.1 404 Not Found\r\n\r\n');
    return;
  }

  const chave = req.headers['sec-websocket-key'];
  if (!chave) {
    socket.end('HTTP/1.1 400 Bad Request\r\n\r\n');
    return;
  }
  const accept = crypto.createHash('sha1').update(chave + GUID).digest('base64');

  /* A cena da conexão. Ver o cabeçalho do arquivo: como as telas não repassam
     a própria querystring ao /ws, caímos no Referer — que é a URL da PÁGINA. */
  let cena = url.searchParams.get('cena');
  let origem = 'query do /ws';
  if (!cena && req.headers.referer) {
    try {
      const daPagina = new URL(req.headers.referer).searchParams.get('cena');
      if (daPagina) { cena = daPagina; origem = 'Referer da página'; }
    } catch { /* Referer estranho: ignora e cai no padrão */ }
  }
  if (!cena || !CENAS[cena]) {
    if (cena) console.log(`[ws] cena "${cena}" não existe; usando "${CENA_PADRAO}"`);
    cena = CENA_PADRAO;
    origem = 'padrão';
  }

  socket.write(
    'HTTP/1.1 101 Switching Protocols\r\n' +
    'Upgrade: websocket\r\n' +
    'Connection: Upgrade\r\n' +
    `Sec-WebSocket-Accept: ${accept}\r\n\r\n`,
  );
  socket.setNoDelay(true);

  const cliente = { socket, cena };
  clientes.add(cliente);
  console.log(`[ws] conectou — cena "${cena}" (${origem}) — ${clientes.size} cliente(s)`);

  const fecha = (motivo) => {
    if (!clientes.delete(cliente)) return;
    console.log(`[ws] saiu (${motivo}) — ${clientes.size} cliente(s)`);
    socket.destroy();
  };

  // O snapshot já vai de cara, para a tela não piscar esperando o próximo tick.
  try { socket.write(frame(0x1, JSON.stringify(CENAS[cena]))); } catch { /* já caiu */ }

  let buf = Buffer.alloc(0);
  socket.on('data', (pedaco) => {
    buf = Buffer.concat([buf, pedaco]);

    for (;;) {
      if (buf.length < 2) return;
      const opcode = buf[0] & 0x0f;
      const mascarado = (buf[1] & 0x80) !== 0;
      let n = buf[1] & 0x7f;
      let off = 2;

      if (n === 126) {
        if (buf.length < off + 2) return;
        n = buf.readUInt16BE(off); off += 2;
      } else if (n === 127) {
        if (buf.length < off + 8) return;
        const grande = buf.readBigUInt64BE(off); off += 8;
        if (grande > 1_000_000n) { fecha('frame absurdo'); return; }
        n = Number(grande);
      }

      let mask = null;
      if (mascarado) {
        if (buf.length < off + 4) return;
        mask = buf.subarray(off, off + 4); off += 4;
      }
      if (buf.length < off + n) return;

      const bruto = Buffer.from(buf.subarray(off, off + n));
      if (mask) for (let i = 0; i < n; i++) bruto[i] ^= mask[i & 3];
      buf = buf.subarray(off + n);

      if (opcode === 0x8) {            // close: devolve o close e encerra
        try { socket.write(frame(0x8, Buffer.alloc(0))); } catch { /* ignora */ }
        fecha('close do cliente');
        return;
      }
      if (opcode === 0x9) {            // ping → pong com o mesmo payload
        try { socket.write(frame(0xa, bruto)); } catch { /* ignora */ }
        continue;
      }
      if (opcode === 0xa) continue;    // pong: nada a fazer

      if (opcode === 0x1) {
        // Comando da tela do operador. Só logamos: este mock não reage a nada.
        console.log(`[ws] comando (cena "${cliente.cena}"): ${bruto.toString('utf8')}`);
      }
    }
  });

  socket.on('error', () => fecha('erro no socket'));
  socket.on('close', () => fecha('socket fechado'));
});

/* O tique. Cada cliente recebe a SUA cena — é o que permite várias abas em
   estados diferentes. Serializamos uma vez por cena por tique. */
setInterval(() => {
  if (clientes.size === 0) return;
  const cache = new Map();
  for (const c of clientes) {
    let pronto = cache.get(c.cena);
    if (!pronto) {
      pronto = frame(0x1, JSON.stringify(CENAS[c.cena]));
      cache.set(c.cena, pronto);
    }
    try { c.socket.write(pronto); } catch { /* o handler de erro remove */ }
  }
}, Math.round(1000 / HZ));

/* ═══════════════════════════════════════════════════════════════════════════
   SUBIDA
   ═══════════════════════════════════════════════════════════════════════════ */

servidor.listen(PORTA, () => {
  const base = `http://localhost:${PORTA}`;
  const faltando = [];
  for (const [rota, arq] of Object.entries(TELAS)) {
    if (rota !== '/' && !fs.existsSync(path.join(DIR_TELAS, arq))) faltando.push(`${rota} (${arq})`);
  }

  console.log('');
  console.log('  mock_ui — árbitro de mentira, só para ver as telas sem ROS');
  console.log(`  ${base}   ·   snapshot a ${HZ} Hz   ·   cena padrão: ${CENA_PADRAO}`);
  console.log('');
  console.log('  A cena vem da querystring da PÁGINA. As telas abrem o /ws sem');
  console.log('  repassar a query, então o servidor lê o Referer do handshake para');
  console.log('  descobrir a cena. Basta colar a URL inteira no navegador:');
  console.log('');
  for (const cena of NOMES_CENAS) {
    console.log(`    ${(cena + ' ').padEnd(17, '·')} ${base}/x1?cena=${cena}`);
  }
  console.log('');
  console.log('  Mesmas cenas nas outras telas (troque /x1 por):');
  console.log(`    ${base}/tv?cena=jogando`);
  console.log(`    ${base}/duelo?cena=jogando`);
  console.log(`    ${base}/operador?cena=jogando`);
  console.log('');
  if (faltando.length) {
    console.log(`  ⚠ telas ausentes neste checkout: ${faltando.join(', ')}`);
    console.log('    (a rota responde 404 com explicação; use as demais por enquanto)');
    console.log('');
  }
  console.log('  Ctrl+C para parar.');
  console.log('');
});

for (const sinal of ['SIGINT', 'SIGTERM']) {
  process.on(sinal, () => {
    for (const c of clientes) c.socket.destroy();
    servidor.close(() => process.exit(0));
    setTimeout(() => process.exit(0), 300).unref();
  });
}
