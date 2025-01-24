#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import yaml
import os
import csv

from APP_config import MAP_NAME

def process_map(input_map_path, output_colored_map_path, yaml_path="mapa_aula.yaml"):
    """
    Procesa un mapa de entrada, genera una imagen con cada región de un color distinto,
    y marca los centroides de cada región en color rojo. También marca el origen en blanco.
    """
    # Cargar el mapa en escala de grises
    image = cv2.imread(input_map_path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(f"No se pudo cargar el mapa en {input_map_path}")

    # Leer el archivo .yaml para obtener resolución y origen
    with open(yaml_path, 'r') as yaml_file:
        yaml_data = yaml.safe_load(yaml_file)
    
    resolution = yaml_data['resolution']  # Resolución en metros por píxel
    origin = yaml_data['origin']  # Origen en coordenadas del mundo real (x, y, theta)
    origin_x, origin_y = origin[0], origin[1]

    # Convertir la imagen a blanco y negro
    _, binary_image = cv2.threshold(image, 240, 255, cv2.THRESH_BINARY)

    # Erosionar para aislar habitaciones
    kernel = np.ones((3, 3), np.uint8)
    eroded_image = binary_image.copy()
    for _ in range(39):
        eroded_image = cv2.erode(eroded_image, kernel)
    for _ in range(39):
        eroded_image = cv2.dilate(eroded_image, kernel)

    # Etiquetar las regiones conectadas
    num_labels, labels = cv2.connectedComponents(eroded_image)

    # Crear una imagen en color para representar las regiones con colores distintos
    colored_image = np.zeros((image.shape[0], image.shape[1], 3), dtype=np.uint8)

    # Calcular los centroides de cada región conectada y pintar la región
    centroids = []
    region_colors = []  # Almacenar los colores asignados a las regiones
    for label in range(1, num_labels):  # Ignorar el fondo (label 0)
        # Crear una máscara para la región actual
        region_mask = (labels == label).astype(np.uint8)

        # Asignar un color aleatorio para cada región
        color = np.random.randint(0, 255, size=3).tolist()  # Genera un color aleatorio
        region_colors.append(color)  # Guardar el color de la región

        # Pintar la región con el color correspondiente
        colored_image[region_mask == 1] = color

        # Calcular los momentos de la región
        moments = cv2.moments(region_mask)
        if moments["m00"] != 0:
            # Calcular las coordenadas del centroide en píxeles
            c_x_pixel = int(moments["m10"] / moments["m00"])
            c_y_pixel = int(moments["m01"] / moments["m00"])

            # Convertir a metros usando la resolución y el origen
            c_x_meter = c_x_pixel * resolution + origin_x
            c_y_meter = origin_y + (image.shape[0] - c_y_pixel) * resolution
            
            # Almacenar el centroide
            centroids.append((c_x_meter, c_y_meter))

            # Marcar el centroide en rojo en la imagen
            cv2.circle(colored_image, (c_x_pixel, c_y_pixel), 5, (0, 0, 255), -1)  # Color rojo en BGR
    
    # Convertir las coordenadas del origen a píxeles
    origin_x_pixel = int(abs(origin_x) / resolution)
    origin_y_pixel = int((image.shape[0] - (-origin_y / resolution)))

    # Marcar el origen en blanco en la imagen
    cv2.circle(colored_image, (origin_x_pixel, origin_y_pixel), 5, (0, 255, 0), -1)  # Color blanco en BGR

    # Detectar los bordes en la imagen original
    edges = cv2.Canny(image, 100, 200)

    # Convertir los bordes a una imagen en color para poder superponerlos
    edges_colored = np.zeros_like(colored_image)
    edges_colored[edges == 255] = [0, 0, 255]  # Bordes en negro

    # Superponer los bordes sobre la imagen coloreada
    final_image = cv2.addWeighted(colored_image, 1, edges_colored, 1, 0)

    # Guardar la imagen resultante
    cv2.imwrite(output_colored_map_path, final_image)
    print(f"Imagen coloreada con centroides, origen y bordes guardada en {output_colored_map_path}")
    
    return labels, num_labels, centroids, region_colors

def save_centroids_and_colors_to_csv(centroids, colors, pgm_paths, output_csv_path):
    """
    Guarda los centroides, colores y rutas de los archivos .pgm en un archivo CSV.
    """
    with open(output_csv_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['X (m)', 'Y (m)', 'Color (R,G,B)', 'Ruta .pgm'])  # Cabecera
        for centroid, color, pgm_path in zip(centroids, colors, pgm_paths):
            writer.writerow([centroid[0], centroid[1], color, pgm_path])
    print(f"Centroides, colores y rutas a archivos .pgm guardados en {output_csv_path}")

def save_restricted_area_overlays(input_map_path, labels, num_labels, output_dir, dilation_kernel_size=(5, 5)):
    """
    Genera un nuevo mapa para cada etiqueta, donde solo esa área está permitida.
    Realiza una dilatación de la máscara para expandir las áreas visibles.
    """
    # Cargar el mapa original en escala de grises
    original_map = cv2.imread(input_map_path, cv2.IMREAD_GRAYSCALE)
    if original_map is None:
        raise FileNotFoundError(f"No se pudo cargar el mapa en {input_map_path}")

    # Crear el directorio de salida si no existe
    os.makedirs(output_dir, exist_ok=True)

    kernel = np.ones(dilation_kernel_size, np.uint8)
    pgm_paths = []  # Lista para almacenar las rutas a los archivos generados

    for label in range(1, num_labels):  # Ignorar el fondo (label 0)
        # Crear una máscara binaria para el área seleccionada
        restricted_mask = (labels == label).astype(np.uint8) * 255

        # Dilatar la máscara para incluir más áreas (paredes o bordes)
        dilated_mask = cv2.dilate(restricted_mask, kernel, iterations=2)

        # Aplicar la máscara dilatada al mapa original
        restricted_map = cv2.bitwise_and(original_map, dilated_mask)

        # Guardar el mapa modificado
        output_path = f"{output_dir}/{MAP_NAME}_area_{label}.pgm"
        pgm_paths.append(output_path)  # Agregar la ruta al archivo generado
        cv2.imwrite(output_path, restricted_map)
        print(f"Mapa con área {label} restringida y dilatada guardado en {output_path}")

    return pgm_paths

# Rutas de los mapas
print(MAP_NAME)
output_dir = "../output_files/restricted_maps"  # Directorio donde guardar los mapas restringidos

input_map = "../nav_maps/" + MAP_NAME + ".pgm"  # Cambia esta ruta por tu mapa original
yaml_path = "../nav_maps/" + MAP_NAME + ".yaml"   # Ruta del archivo .yaml
output_colored_map = "../output_files/" + MAP_NAME + ".png"  # Ruta para guardar la imagen coloreada
output_colored_csv_path = "../output_files/" + MAP_NAME + ".csv"   # Ruta para guardar el CSV de centroides, colores y rutas .pgm

# Procesar el mapa para etiquetar áreas
labels, num_labels, centroids, region_colors = process_map(input_map, output_colored_map, yaml_path=yaml_path)

# Generar mapas con restricciones para cada área, aplicando dilatación, y obtener las rutas
pgm_paths = save_restricted_area_overlays(input_map, labels, num_labels, output_dir, dilation_kernel_size=(5, 5))

# Guardar los centroides, colores y rutas de archivos en un archivo CSV
save_centroids_and_colors_to_csv(centroids, region_colors, pgm_paths, output_colored_csv_path)