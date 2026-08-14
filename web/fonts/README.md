# Fontes

As três letras do manual da marca, recortadas para a web. Gere de novo com
`tools/make_fonts.py`, que documenta o recorte e de onde veio cada arquivo.

Ficam versionadas no repositório de propósito: a feira não tem rede garantida,
e fonte que não carrega derruba a identidade do telão na hora errada. São
89 KB no total — menos que um único frame do vídeo da calibração.

| arquivo | face | papel |
|---|---|---|
| `display.woff2` | Advent Pro, **variável** (`wdth` 100–200, `wght` 100–900) | wordmark, placar, cronômetro, números grandes |
| `sans-regular.woff2` | Poppins 400 | corpo de texto |
| `sans-semibold.woff2` | Poppins 600 | título, rótulo |
| `sans-bold.woff2` | Poppins 700 | ênfase |
| `sans-black.woff2` | Poppins 900 | número do placar, lido a dez metros |
| `mono-medium.woff2` | Fira Mono Medium | telemetria, tópicos ROS, números de ferramenta |

**A Advent Pro é uma fonte variável e é usada como tal.** Um arquivo de 49 KB
cobre do wordmark (largura normal) ao placar do telão (`font-stretch: 175%`,
peso 700) e à etiqueta estreita. Instanciar cada largura num arquivo separado
custaria três vezes mais bytes pelo mesmo resultado. Quem mexer aqui: não passe
`varLib.instancer` no arquivo, ou os eixos somem e o placar volta à largura
normal sem erro nenhum no log.

**Por que a Poppins e não a Bowlby One:** a revisão de design do site trocou
uma pela outra "para trazer mais seriedade", e o telão segue a mesma decisão.
O manual é antigo — vale como decisão registrada, não como autoridade viva.

## Símbolo aqui é SVG, não caractere

Nenhuma das fontes da marca tem seta, check, aviso ou bolinha — medido, está na
tabela no topo do `tools/make_fonts.py`. Escrever `→` ou `✓` no HTML faz o
navegador cair numa fonte do sistema, e num computador de feira que ninguém
auditou isso é retângulo vazio na tela grande.

Então: **ícone é `<svg>` do `vss.css`.** O que a fonte tem de verdade e pode ser
escrito como texto é `× · • – — ° º ª` — conferido na Advent Pro, que é quem
desenha o placar.

## Licença

Todas sob a **SIL Open Font License 1.1**, que permite redistribuir os arquivos.
Os textos estão aqui do lado: `OFL-AdventPro.txt`, `OFL-Poppins.txt`,
`OFL-Fira.txt`. <https://openfontlicense.org>

Origem: Advent Pro e Poppins do material de identidade da equipe / Google Fonts;
Fira Mono de `/usr/share/fonts/opentype/fira` (pacote Debian `fonts-firacode`).
