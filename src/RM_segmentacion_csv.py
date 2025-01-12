import cv2
import numpy as np
import yaml
import os
import csv

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
    for _ in range(15):
        eroded_image = cv2.erode(eroded_image, kernel)
    for _ in range(15):
        eroded_image = cv2.dilate(eroded_image, kernel)

    # Etiquetar las regiones conectadas
    num_labels, labels = cv2.connectedComponents(eroded_image)

    # Crear una imagen en color para representar las regiones con colores distintos
    colored_image = np.zeros((image.shape[0], image.shape[1], 3), dtype=np.uint8)

    # Calcular los centroides de cada región conectada y pintar la región
    centroids = []
    for label in range(1, num_labels):  # Ignorar el fondo (label 0)
        # Crear una máscara para la región actual
        region_mask = (labels == label).astype(np.uint8)

        # Asignar un color aleatorio para cada región
        color = np.random.randint(0, 255, size=3).tolist()  # Genera un color aleatorio

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
    
    return labels, num_labels, centroids

def save_centroids_to_csv(centroids, output_csv_path):
    """
    Guarda los centroides en un archivo CSV.
    """
    with open(output_csv_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['X (m)', 'Y (m)'])  # Cabecera
        for centroid in centroids:
            writer.writerow(centroid)
    print(f"Centroides guardados en {output_csv_path}")

def traverse_centroids(centroids):
    """
    Realiza un recorrido por los centroides.
    """
    print("Recorrido por los centroides (en metros):")
    for idx, centroid in enumerate(centroids):
        print(f"Paso {idx + 1}: {centroid}")

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

    for label in range(1, num_labels):  # Ignorar el fondo (label 0)
        # Crear una máscara binaria para el área seleccionada
        restricted_mask = (labels == label).astype(np.uint8) * 255

        # Dilatar la máscara para incluir más áreas (paredes o bordes)
        dilated_mask = cv2.dilate(restricted_mask, kernel, iterations=2)

        # Aplicar la máscara dilatada al mapa original
        restricted_map = cv2.bitwise_and(original_map, dilated_mask)

        # Guardar el mapa modificado
        output_path = f"{output_dir}/restricted_area_{label}.pgm"
        cv2.imwrite(output_path, restricted_map)
        print(f"Mapa con área {label} restringida y dilatada guardado en {output_path}")

# Rutas de los mapas
input_map = "casa_3.pgm"  # Cambia esta ruta por tu mapa original
output_colored_map = "mapa_areas.png"  # Ruta para guardar la imagen coloreada
yaml_path = "casa_3.yaml"  # Ruta del archivo .yaml
output_csv_path = "centroides.csv"  # Ruta para guardar el CSV de centroides
output_dir = "restricted_maps"  # Directorio donde guardar los mapas restringidos

# Procesar el mapa para etiquetar áreas
labels, num_labels, centroids = process_map(input_map, output_colored_map, yaml_path=yaml_path)

# Guardar los centroides en un archivo CSV
save_centroids_to_csv(centroids, output_csv_path)

# Realizar un recorrido por los centroides
traverse_centroids(centroids)

# Generar mapas con restricciones para cada área, aplicando dilatación
save_restricted_area_overlays(input_map, labels, num_labels, output_dir, dilation_kernel_size=(5, 5))

# Mostrar los centroides calculados
print("Centroides de las regiones conectadas (en metros):")
for centroid in centroids:
    print(centroid)

