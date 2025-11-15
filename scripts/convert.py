import json
import random
import argparse
from pathlib import Path


def convert(path: Path, keep: int | None = None, dest: Path | None = None):
    with open(path, 'rb') as file:
        tests = json.load(file)

    if keep is not None:
        tests = random.sample(tests, keep)

    out = bytearray()
    out += len(tests).to_bytes(2, 'little')

    for test in tests:
        used_addresses = [addr for addr, _, _ in test['cycles']]
        used_memory = [(addr, data) for addr, data in test['initial']['ram'] if addr in used_addresses]

        for state in ['initial', 'final']:
            out += test[state]['pc'].to_bytes(2, 'little')
            out += test[state]['a'].to_bytes()
            out += test[state]['x'].to_bytes()
            out += test[state]['y'].to_bytes()
            out += test[state]['s'].to_bytes()
            out += test[state]['p'].to_bytes()

        out += len(used_memory).to_bytes()
        for address, data in used_memory:
            out += address.to_bytes(2, 'little')
            out += data.to_bytes()

        out += len(test['cycles']).to_bytes()
        for address, data, kind in test['cycles']:
            kind = ord('r' if kind == 'read' else 'w')
            out += address.to_bytes(2, 'little')
            out += data.to_bytes()
            out += kind.to_bytes()

    if dest:
        dest.mkdir(parents=True, exist_ok=True)
        dest = dest / path.with_suffix('.bin').name
    else:
        dest = path.with_suffix('.bin')

    with open(dest, 'wb') as file:
        file.write(out)


def main():
    parser = argparse.ArgumentParser(description='Convert CPU opcode tests to binary.')
    parser.add_argument('path', help='path to the file or directory containing opcode tests')
    parser.add_argument('-k', '--keep', type=int, help='number of random tests per opcode to include in the binary')
    parser.add_argument('-d', '--dest', type=Path, help='directory to save the output binary files')
    args = parser.parse_args()

    if args.keep is not None:
        if not 1 <= args.keep <= 10000:
            raise Exception('`keep` value must be in range [1, 10000]')

    path = Path(args.path)

    if path.is_file():
        convert(path, args.keep, args.dest)
    elif path.is_dir():
        for path in path.rglob('*.json'):
            convert(path, args.keep, args.dest)
            print(f'Converted {path.name}')
    else:
        raise Exception('`path` must be a file or directory')


if __name__ == '__main__':
    main()
