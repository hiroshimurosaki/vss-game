#!/usr/bin/env python3
"""Testa o rádio e os robôs sem subir o ROS.

Fala direto com o tx_bridge pela serial, montando o mesmo pacote de 14 bytes que
o nó radio_communication monta. Serve para responder rápido "o problema é o robô
ou é o ROS?" — que é a pergunta que mais custa tempo em dia de competição.

Usa só a stdlib (termios), então não precisa de pyserial.

Exemplos:
    # roda o robô 0 para frente por 2 segundos
    ./tools/radio_test.py --id 0 --left 0.5 --right 0.5 --duration 2

    # gira o robô 1 no lugar
    ./tools/radio_test.py --id 1 --left -0.4 --right 0.4 --duration 3

    # varre os IDs 0..3 um por vez, para descobrir qual robô tem qual ID
    ./tools/radio_test.py --sweep

    # teclado: setas dirigem, espaço para, q sai
    ./tools/radio_test.py --id 0 --interactive
"""

import argparse
import os
import struct
import sys
import termios
import time
import tty

START_BYTE = 0x14
PACKET_FMT = '<BffiB'  # start_byte, motor1, motor2, robot_id, checksum
PACKET_SIZE = struct.calcsize(PACKET_FMT)

assert PACKET_SIZE == 14, f'pacote deveria ter 14 bytes, tem {PACKET_SIZE}'


def build_packet(robot_id: int, left: float, right: float) -> bytes:
    """Monta o pacote com o mesmo layout e checksum do firmware."""
    left = max(-1.0, min(1.0, left))
    right = max(-1.0, min(1.0, right))

    body = struct.pack('<ffi', left, right, robot_id)

    # XOR de motor1 até robot_id. Não inclui start_byte nem o checksum.
    checksum = 0
    for byte in body:
        checksum ^= byte

    return struct.pack('<B', START_BYTE) + body + struct.pack('<B', checksum)


def open_serial(port: str, baud: int = 115200) -> int:
    """Abre e configura a serial em modo raw, sem pyserial."""
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY)

    baud_const = getattr(termios, f'B{baud}')

    attrs = termios.tcgetattr(fd)
    iflag, oflag, cflag, lflag, ispeed, ospeed, cc = attrs

    iflag &= ~(termios.IXON | termios.IXOFF | termios.IXANY | termios.ICRNL |
               termios.INLCR | termios.IGNCR | termios.ISTRIP | termios.BRKINT)
    oflag &= ~termios.OPOST
    lflag &= ~(termios.ICANON | termios.ECHO | termios.ECHOE | termios.ISIG)
    cflag &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE)
    cflag |= termios.CS8 | termios.CLOCAL | termios.CREAD

    cc[termios.VMIN] = 0
    cc[termios.VTIME] = 0

    termios.tcsetattr(fd, termios.TCSANOW,
                      [iflag, oflag, cflag, lflag, baud_const, baud_const, cc])

    # Abrir a porta reseta o Nano via DTR; ele leva ~2 s para sair do bootloader.
    # Sem esta espera os primeiros comandos somem e parece que o rádio não funciona.
    time.sleep(2.0)
    termios.tcflush(fd, termios.TCIOFLUSH)

    return fd


def drive(fd: int, robot_id: int, left: float, right: float, duration: float, rate: float = 30.0):
    """Mantém um comando por `duration` segundos.

    Repetir é obrigatório: o firmware para os motores se ficar 1 s sem pacote.
    """
    packet = build_packet(robot_id, left, right)
    period = 1.0 / rate
    deadline = time.monotonic() + duration

    while time.monotonic() < deadline:
        os.write(fd, packet)
        time.sleep(period)


def stop(fd: int, robot_id: int):
    packet = build_packet(robot_id, 0.0, 0.0)
    for _ in range(5):
        os.write(fd, packet)
        time.sleep(0.02)


def run_sweep(fd: int, duration: float, speed: float):
    """Move um robô de cada vez, para mapear qual chassi responde a qual ID."""
    for robot_id in range(4):
        print(f'  ID {robot_id}: girando por {duration:.0f}s... ', end='', flush=True)
        drive(fd, robot_id, -speed, speed, duration)
        stop(fd, robot_id)
        print('feito')
        time.sleep(0.5)


def run_interactive(fd: int, robot_id: int, speed: float):
    """Setas dirigem, espaço para, q sai."""
    print('  setas = dirigir | espaço = parar | q = sair')

    old = termios.tcgetattr(sys.stdin)
    left = right = 0.0
    last_input = time.monotonic()

    try:
        tty.setcbreak(sys.stdin.fileno())
        os.set_blocking(sys.stdin.fileno(), False)

        while True:
            ch = sys.stdin.read(1)

            if ch:
                last_input = time.monotonic()

                if ch == 'q':
                    break
                if ch == ' ':
                    left = right = 0.0
                elif ch == '\x1b':  # sequência de escape das setas
                    seq = sys.stdin.read(2)
                    if seq == '[A':
                        left = right = speed
                    elif seq == '[B':
                        left = right = -speed
                    elif seq == '[C':
                        left, right = speed, -speed
                    elif seq == '[D':
                        left, right = -speed, speed

            # Solta os motores se ninguém tocar em nada por meio segundo.
            if time.monotonic() - last_input > 0.5:
                left = right = 0.0

            os.write(fd, build_packet(robot_id, left, right))
            time.sleep(1.0 / 30.0)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        stop(fd, robot_id)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--id', type=int, default=0, help='robot_id alvo')
    parser.add_argument('--left', type=float, default=0.0, help='roda esquerda, -1.0 a 1.0')
    parser.add_argument('--right', type=float, default=0.0, help='roda direita, -1.0 a 1.0')
    parser.add_argument('--duration', type=float, default=2.0, help='segundos')
    parser.add_argument('--speed', type=float, default=0.5, help='velocidade do sweep/interativo')
    parser.add_argument('--sweep', action='store_true', help='testa os IDs 0..3 em sequência')
    parser.add_argument('--interactive', action='store_true', help='dirige pelo teclado')
    parser.add_argument('--dry-run', action='store_true', help='só imprime o pacote, não abre a serial')

    args = parser.parse_args()

    packet = build_packet(args.id, args.left, args.right)
    print(f'pacote ({len(packet)} bytes): {packet.hex(" ")}')

    if args.dry_run:
        return 0

    if not os.path.exists(args.port):
        print(f'ERRO: {args.port} não existe. Portas disponíveis:')
        for entry in sorted(os.listdir('/dev')):
            if entry.startswith(('ttyUSB', 'ttyACM')):
                print(f'  /dev/{entry}')
        return 1

    print(f'abrindo {args.port} a {args.baud} baud (aguardando reset do Arduino)...')
    fd = open_serial(args.port, args.baud)

    try:
        if args.sweep:
            run_sweep(fd, args.duration, args.speed)
        elif args.interactive:
            run_interactive(fd, args.id, args.speed)
        else:
            print(f'robô {args.id}: left={args.left} right={args.right} por {args.duration}s')
            drive(fd, args.id, args.left, args.right, args.duration)
            stop(fd, args.id)
            print('feito')
    except KeyboardInterrupt:
        stop(fd, args.id)
        print('\ninterrompido')
    finally:
        os.close(fd)

    return 0


if __name__ == '__main__':
    sys.exit(main())
