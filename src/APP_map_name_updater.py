#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import re

def update_map_name_in_file(file_path, new_map_name):
    # Expresión regular para encontrar la línea MAP_NAME = "..."
    pattern = r'^(MAP_NAME\s*=\s*")[^"]*(")$'

    # Leer el contenido del archivo
    with open(file_path, 'r') as file:
        lines = file.readlines()

    # Modificar la línea que contiene MAP_NAME
    with open(file_path, 'w') as file:
        for line in lines:
            if re.match(pattern, line.strip()):
                line = f'MAP_NAME = "{new_map_name}"\n'
            file.write(line)

    print(f"MAP_NAME actualizado a '{new_map_name}' en {file_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Actualizar MAP_NAME en un archivo Python.")
    parser.add_argument("file_path", help="Ruta del archivo Python a modificar")
    parser.add_argument("map_name", help="Nuevo valor para MAP_NAME")
    args = parser.parse_args()

    update_map_name_in_file(args.file_path, args.map_name)
